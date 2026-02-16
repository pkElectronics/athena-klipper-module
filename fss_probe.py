# Force Sensor Probe support for MSLA Printers
#
# Copyright (C) 2022  Pascal Wistinghausen (pascal.wistinghausen@ib-wistinghausen.de)
# Based on previous works by Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
from typing import Callable, Tuple
import pins
from webhooks import Sentinel

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
"""


class PrinterFssProbe:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe = mcu_probe
        self.lift_speed = config.getfloat('lift_speed', 10.0, above=0.)
        self.lift_amount = config.getfloat('lift_amount', 10.0, above=0.)
        self.min_lift_distance = config.getfloat('min_lift_distance', 2.0, above=.5)
        self.smart_dip_segment_resolution = config.getfloat("smart_dip_segment_resolution", 0.005, above=0.)
        self.full_lift_speed = None

        self.buildplate_area = 120*210 #make configurable in the future

        self.enable_powermgmt = config.getboolean("enable_uvled_powermgmt", True)

        if self.enable_powermgmt:
            self.powermanaged_heater_name = config.get("uvled_powermgmt_heater_name","heater_generic resin_heater")

        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")

        self.exposure_calibration = 1.0

        self.last_exposure_time = 0
        self.last_exposure_power = 0
        self.last_exposure_pre_delay = 0
        self.last_exposure_post_delay = 0
        self.last_gcmd = None
        self.last_resin_level = 5.0

        self.resin_temp_setpoint = 0.0
        self.resinheater = None

        self.peelmode = "minimal"

        self.expose_processing_delay = 0.300

        self.exposure_active_flag = False
        self.uvled_pwm_cutoff_value = 0.25

        self.z_offset = 0.0

        self.reactor = self.printer.get_reactor()



        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)

        # Register z_virtual_endstop pin
        self.printer.lookup_object('pins').register_chip('fss_probe', self)

        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command('MOVE_PLATE_FSS', self.cmd_ATHENA_PROBE_UPWARDS,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('ATHENA_PROBE_UPWARDS', self.cmd_ATHENA_PROBE_UPWARDS,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('ATHENA_PROBE_DOWNWARDS', self.cmd_ATHENA_PROBE_DOWNWARDS,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('ATHENA_PROBE_RESINLEVEL', self.cmd_ATHENA_PROBE_RESINLEVEL,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('ATHENA_OVERRIDE_RESINLEVEL', self.cmd_ATHENA_OVERRIDE_RESINLEVEL,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('ATHENA_MOVE', self.cmd_ATHENA_PROBE_DOWNWARDS,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('QUERY_FSS', self.cmd_QUERY_FSS,
                                    desc=self.cmd_QUERY_FSS_help)

        self.gcode.register_command('EXPOSE', self.cmd_EXPOSE,
                                    desc=self.cmd_EXPOSE_help)

        self.gcode.register_command('SET_EXPOSE_CALIBRATION', self.cmd_SET_EXPOSE_CALIBRATION,
                                    desc=self.cmd_SET_EXPOSE_CALIBRATION)

        self.gcode.register_command('SET_Z_OFFSET', self.cmd_SET_Z_OFFSET,
                                    desc=self.cmd_SET_Z_OFFSET_help)

        self.gcode.register_command('ATHENA_SET_PEELMODE_MINIMAL', self.cmd_ATHENA_SET_PEELMODE_MINIMAL)

        self.gcode.register_command('ATHENA_SET_PEELMODE_FULL', self.cmd_ATHENA_SET_PEELMODE_FULL)

        self.gcode.register_command('ATHENA_SET_MINIMUM_LIFT_DISTANCE', self.cmd_ATHENA_SET_MINIMUM_LIFT_DISTANCE)

        self.gcode.register_command('ATHENA_SET_FULL_LIFT_SPEED', self.cmd_ATHENA_SET_FULL_LIFT_SPEED)

        self.gcode.register_command('ATHENA_SMART_DIP', self.cmd_ATHENA_SMART_DIP)
        self.gcode.register_command('ATHENA_SMART_PEEL', self.cmd_ATHENA_SMART_PEEL)

    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("F", self.lift_speed, above=0.)
        return self.lift_speed

    def _move(self, position, speed):
        toolhead = self.printer.lookup_object('toolhead')
        position[2] += self.z_offset
        toolhead.move(position,speed)

    def _get_position(self):
        toolhead = self.printer.lookup_object('toolhead')
        p = toolhead.get_position()
        p[2] -= self.z_offset
        return p

    def _probe(self, speed, amount):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")

        phoming = self.printer.lookup_object('homing')
        pos = self._get_position()
        opos = pos[2]
        pos[2] += amount + self.z_offset
        epos = [pos[0], pos[1], pos[2]]
        try:
            epos = phoming.probing_move(self.mcu_probe, pos, speed)

            epos[2] = epos[2] - opos
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
                raise self.printer.command_error(reason)

            elif "No trigger on probe after full movement" in reason:
                # in our case this is not an error but desired behaviorcr
                epos = self._get_position()
                epos[2] = amount

            elif "Probe triggered prior to movement" in reason:
                self._move(pos, speed)
                epos = self._get_position()
                epos[2] = amount

            else:
                raise self.printer.command_error(reason)

        return epos[:3]

    def run_probe_upwards(self, gcmd):
        lift_amount = gcmd.get_float("Z", self.lift_amount, minval=0.)
        lift_speed = gcmd.get_float("F", self.lift_speed, above=0.) / 60
        stage1_lift_distance = 0.2
        toolhead = self.printer.lookup_object('toolhead')

        position = self._get_position()

        stage1_position = position.copy()
        stage1_position[2] += stage1_lift_distance

        kinematics = toolhead.get_kinematics()

        saved_accel_decel = kinematics.get_accel_decel()
        stage1_accel_decel = saved_accel_decel.copy()
        stage1_accel_decel["peel_accel"] = .1
        stage1_accel_decel["peel_decel"] = 1000
        kinematics.set_accel_decel(stage1_accel_decel)

        self._move(stage1_position,lift_speed)

        stage1_accel_decel["peel_accel"] = 1000
        kinematics.set_accel_decel(stage1_accel_decel)

        pos = self._probe(lift_speed, lift_amount - stage1_lift_distance)

        kinematics.set_accel_decel(saved_accel_decel)

        if self.peelmode == "minimal":
            if pos[2] < self.min_lift_distance:
                logging.info("Minimum lift distance not reached: %f required: %f", pos[2], self.min_lift_distance)
                pos_actual = self._get_position()
                remaining_move = self.min_lift_distance - pos[2]

                if remaining_move > 0.1:
                    pos_actual[2] += remaining_move
                    self._move(pos_actual, lift_speed)
                    pos[2] = self.min_lift_distance
                    toolhead.wait_moves()
                else:
                    logging.info("Skipping due to hysteresis")

        elif self.peelmode == "full":
            logging.info("Peel finished after %f", pos[2])
            pos_actual = self._get_position()
            already_travelled = stage1_lift_distance + pos[2]
            remaining_move = lift_amount - already_travelled

            if remaining_move > 0.1:

                kin = toolhead.get_kinematics()
                current_accel_decel = kin.get_accel_decel()
                new_accel_decel = current_accel_decel.copy()
                new_accel_decel["peel_accel"] = new_accel_decel["peel_decel"]
                kin.set_accel_decel(new_accel_decel)

                if self.min_lift_distance - already_travelled > 0.1:
                    remaining_min_lift_move = self.min_lift_distance - already_travelled

                    logging.info("Doing slow min lift first: %f mm",remaining_min_lift_move)

                    pos_actual[2] += remaining_min_lift_move
                    remaining_move -= remaining_min_lift_move
                    self._move(pos_actual, lift_speed)


                if self.full_lift_speed is None or self.full_lift_speed == 0 :
                    speed = lift_speed*2
                else:
                    speed = self.full_lift_speed

                pos_actual[2] += remaining_move
                self._move(pos_actual, speed)

                kin.set_accel_decel(current_accel_decel)
                pos[2] = lift_amount

                toolhead.wait_moves()
            else:
                logging.info("Skipping due to hysteresis")

        return pos

    def _calc_peel_v2(self,s2_minspeed, s2_maxarea, s2_maxspeed, s2_minarea,area):
        coeff = (s2_maxspeed - s2_minspeed) / (s2_minarea - s2_maxarea)
        speed = coeff * (area - s2_maxarea) + s2_minspeed
        return max(min(speed,s2_maxspeed),s2_minspeed)

    def _calc_peel_v1(self,s1_minspeed, s1_maxarea, s1_maxspeed, s1_minarea,area):
        coeff = (s1_maxspeed - s1_minspeed) / (s1_minarea - s1_maxarea)
        speedv1 = coeff * (area - s1_maxarea) + s1_minspeed
        return max(min(speedv1,s1_maxspeed),s1_minspeed)

    def _calc_peel_l1(self,s1_minlift, s1_maxarea, s1_maxlift, s1_minarea,area):
        coeff = (s1_maxlift - s1_minlift) / (s1_maxarea - s1_minarea)
        lifts1 = coeff * (area - s1_minarea) + s1_minlift
        return max(min(lifts1,s1_maxlift),s1_minlift)

    def smart_peel(self, gcmd):
        lift_total = gcmd.get_float("LIFT_TOTAL", above=0.)
        stage1_minlift = gcmd.get_float("STAGE1_MINLIFT", above=0.)
        stage1_maxlift = gcmd.get_float("STAGE1_MAXLIFT", above=0.)
        stage1_minspeed = gcmd.get_float("STAGE1_MINSPEED", above=0.) / 60
        stage1_maxspeed = gcmd.get_float("STAGE1_MAXSPEED", above=0.) / 60
        stage1_accel = gcmd.get_float("STAGE1_ACCEL", above=0.)
        interstage_accel = gcmd.get_float("INTERSTAGE_ACCEL", above=0.)

        stage1_maxarea = gcmd.get_float("STAGE1_MAXAREA", above=0.)
        stage1_minarea = gcmd.get_float("STAGE1_MINAREA", above=0.)

        stage2_minspeed = gcmd.get_float("STAGE2_MINSPEED", above=0.) /60
        stage2_maxarea = gcmd.get_float("STAGE2_MAXAREA", above=0.)

        stage2_maxspeed = gcmd.get_float("STAGE2_MAXSPEED", above=0.) /60
        stage2_minarea = gcmd.get_float("STAGE2_MINAREA", above=0.)

        surface_area_mm2 = gcmd.get_float("SURFACEAREA", above=0.) #from print data

        stage1_lift = self._calc_peel_l1(stage1_minlift,stage1_maxarea,stage1_maxlift,stage1_minarea,surface_area_mm2)

        stage1_speed = self._calc_peel_v1(stage1_minspeed,stage1_maxarea,stage1_maxspeed,stage1_minarea,surface_area_mm2)

        stage2_speed = self._calc_peel_v2(stage2_minspeed,stage2_maxarea,stage2_maxspeed,stage2_minarea,surface_area_mm2)
        logging.info(f"Computed S2 Speed: {stage2_speed}mm/s | {stage2_speed*60}mm/min")
        gcmd.respond_raw(f"Smart Peel Computed Values - S1 Lift Distance: {stage1_lift}mm | S1 Lift Speed: {stage1_speed*60}mm/min | S2 Lift Speed: {stage2_speed*60}mm/min")

        toolhead = self.printer.lookup_object('toolhead')

        position = self._get_position()

        stage1_position = position.copy()
        stage1_position[2] += stage1_lift

        stage2_position = position.copy()
        stage2_position[2] += lift_total

        kinematics = toolhead.get_kinematics()

        saved_accel_decel = kinematics.get_accel_decel()
        stage1_accel_decel = saved_accel_decel.copy()
        stage1_accel_decel["peel_accel"] = stage1_accel
        stage1_accel_decel["peel_decel"] = stage1_accel

        stage2_accel_decel = saved_accel_decel.copy()
        stage2_accel_decel["peel_accel"] = interstage_accel

        kinematics.set_accel_decel(stage1_accel_decel)

        self._move(stage1_position,stage1_speed)

        kinematics.set_accel_decel(stage2_accel_decel)

        self._move(stage2_position, stage2_speed)

        toolhead.wait_moves()

        kinematics.set_accel_decel(saved_accel_decel)


        return self._get_position()



    def _calc_v(self,P_mpa,dl,u,A,d_d):
        v1 = ((P_mpa*math.pi*2*(d_d+dl)**3)/(3*u*A))*(600*(dl+d_d)/(math.sqrt(A/math.pi)))
        v = min(max(v1,0.01),10)
        return v


    def smart_dip(self, gcmd):
        #this version is an approximation of a constant pressure velocity profile using a similar scheme as a g2 command
        logging.info(f"into smart_dip command")
        mPa_to_gfmm2 = .0000102
        viscosity_cps = gcmd.get_float("VISCOSITY", above=0.) #from resin profile
        resin_level_mm = self.last_resin_level
        surface_area_mm2 = gcmd.get_float("SURFACEAREA", minval=0.) #from print data
        buildplate_area_mm2 = self.buildplate_area
        layerheight_mm = gcmd.get_float("LAYERHEIGHT", above=0.) # from slice data
        pressure_mpa = gcmd.get_float("PRESSURE", above=0.) #from resin profile
        pressure_gfmm2 = pressure_mpa * mPa_to_gfmm2

        target_position = gcmd.get_float("TARGET", minval=0.)
        target_speed = gcmd.get_float("SPEED", minval=0.)

        if surface_area_mm2 > 40000 or surface_area_mm2 == 0:
            surface_area_mm2 = 210*120*0.2

        # constant factors for generating velocity profile
        resolution = self.smart_dip_segment_resolution # similar to g2 gcode
        vmin = 0.05 # minimum velocity 0.3mm/min
        vmax = target_speed / 60   # maximum velocity 600mm/min
        max_force = 20000 #maximum force a retract move will try to achieve
        viscosity_coefficient = 60   #constant for movements outside of squeezing flow regime
        e_plate_area = buildplate_area_mm2 * 0.75 # the circular eqiuvalent area which produces the same constant pressure curve
        pressure_mpa_maxforce = (max_force/e_plate_area)/mPa_to_gfmm2    #pressure on build plate corresponding to maximum force
        ilaC = 0.5 #Initial layer accuracy coefficient, determines the amount of overshoot on the first layers for better layer thickness accuracy, increases first layer time


        toolhead = self.printer.lookup_object('toolhead')
        pos = self._get_position()
        dip_amount = pos[2] - target_position
        d_d = max(0.1,(0.218*((surface_area_mm2*pressure_gfmm2)**0.821)) / 1000)    # maximum arm deflection from retract force on layer area
        d_d2 = max(0.1,(0.218*((max_force)**0.821)) / 1000) # maximum arm deflection due to force on build plate
        logging.info(f"Actual Dip Amount {dip_amount}, Target Z {target_position}")

        if target_position < 0.5:
            deflection = ((d_d2*ilaC)*(1-(target_position*2))+((d_d*ilaC)*(target_position*2)))
        else:
            deflection = (d_d*ilaC)

        deflection = 0

        segments = max(1., math.floor((dip_amount+deflection) / resolution))
        logging.info(f"Deflection {deflection}, Segments {segments}")

        if target_position < resin_level_mm:
            for i in range(1, int(segments) + 1):
                step_pos = [0. , 0. , 0. , 0.]
                step_pos[2] =  pos[2] - (i * resolution)
                di = dip_amount + layerheight_mm - (i*resolution)
                # velocity = minimum of velocity for maximum part pressure or velocity corresponding with maximum force
                v1 = (3*(pressure_mpa*math.pi*2*(d_d+di)**3)/(3*viscosity_cps*surface_area_mm2))*(1 + (viscosity_coefficient*math.atan(((di)+d_d)/(math.sqrt(surface_area_mm2/math.pi)))))
                v2 = ((pressure_mpa_maxforce*math.pi*2*(d_d2+di)**3)/(3*viscosity_cps*e_plate_area))*(1 + (viscosity_coefficient*math.atan(((di)+d_d2)/(math.sqrt(e_plate_area/math.pi)))))
                v3 = min(v1,v2)
                velocity = min(max(v3,vmin),vmax)
                self._move(step_pos,velocity)

        else:
            for i in range(1, int(segments) + 1):
                step_pos = [0. , 0. , 0. , 0.]
                step_pos[2] =  pos[2] - (i * resolution)
                di = dip_amount + layerheight_mm - (i*resolution)
                v1 = (3*(pressure_mpa*math.pi*2*(d_d+di)**3)/(3*viscosity_cps*surface_area_mm2))*(1 + (viscosity_coefficient*math.atan(((di)+d_d)/(math.sqrt(surface_area_mm2/math.pi)))))
                velocity = min(max(v1,vmin),vmax)
                self._move(step_pos,velocity)

        #self._move([0,0,target_position,0],5)
        toolhead.wait_moves()
        pos = self._get_position()

        return pos

    def run_probe_downwards(self, gcmd):
        dip_speed = gcmd.get_float("F", self.lift_speed, above=0.) / 60
        dip_amount = gcmd.get_float("Z", 0, minval=0.)
        move_absolute = gcmd.get_int("ABS",0, minval=0, maxval=1)
        toolhead = self.printer.lookup_object('toolhead')
        pos = self._get_position()

        if move_absolute == 1:
            dip_amount = (pos[2] - dip_amount)

        if dip_amount == 0:
            dip_amount = -1*pos[2]
        else:
            dip_amount = -1 * dip_amount

        pos = self._probe(dip_speed, dip_amount)  # probe to zero

        pos = self._get_position()

        return pos

    cmd_PROBE_help = "Probe Z-height at current XY position"

    def cmd_ATHENA_SMART_DIP(self,gcmd):
        self.smart_dip(gcmd)
        gcmd.respond_raw("Z_move_comp")

    def cmd_ATHENA_SMART_PEEL(self,gcmd):
        self.smart_peel(gcmd)
        gcmd.respond_raw("Z_move_comp")


    def cmd_ATHENA_PROBE_UPWARDS(self, gcmd):
        pos = self.run_probe_upwards(gcmd)
        gcmd.respond_raw("Z_move_comp")
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]

    def cmd_ATHENA_PROBE_DOWNWARDS(self, gcmd):
        pos = self.run_probe_downwards(gcmd)
        gcmd.respond_raw("Z_move_comp")
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]

    def cmd_ATHENA_PROBE_RESINLEVEL(self, gcmd):
        pos = self.run_probe_downwards(gcmd)
        gcmd.respond_raw("Z_move_comp")
        gcmd.respond_raw("ResinLevel:%.2f" % (pos[2],))
        self.last_resin_level = pos[2]
        self.last_z_result = pos[2]

    def cmd_ATHENA_OVERRIDE_RESINLEVEL(self, gcmd):
        self.last_resin_level = gcmd.get_float("LEVEL", above=0.)  # from resin profile

    def cmd_ATHENA_SET_PEELMODE_MINIMAL(self, gcmd):
        self.peelmode="minimal"

    def cmd_ATHENA_SET_PEELMODE_FULL(self, gcmd):
        self.peelmode="full"

    def cmd_ATHENA_SET_MINIMUM_LIFT_DISTANCE(self, gcmd):
        self.min_lift_distance = gcmd.get_float("VALUE", self.lift_amount, minval=0.)

    def cmd_ATHENA_SET_FULL_LIFT_SPEED(self, gcmd):
        self.full_lift_speed = gcmd.get_float("VALUE", self.lift_speed * 60, minval=0.) / 60


    cmd_QUERY_FSS_help = "Return the status of the z-probe"

    def cmd_QUERY_FSS(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("fss input: %s" % (["open", "TRIGGERED"][not not res],))

    def exposure_timing_callback(self, print_time):
        reactor_time = self.reactor.monotonic()

        self.reactor.register_callback(self.exposure_done_callback, reactor_time+self.last_exposure_time+self.expose_processing_delay*2+self.last_exposure_pre_delay+self.last_exposure_post_delay)

        self.ledpwm.mcu_pin.set_pwm(print_time+self.expose_processing_delay+self.last_exposure_pre_delay, self.last_exposure_power)
        self.ledpwm.mcu_pin.set_pwm(print_time+self.expose_processing_delay+self.last_exposure_pre_delay+self.last_exposure_time, 0)

    def exposure_done_callback(self, print_time):

        if self.enable_powermgmt and self.resin_temp_setpoint != 0.0:
            self.resinheater.set_temp(self.resin_temp_setpoint)

        self.exposure_active_flag = False
        self.last_gcmd.respond_raw("Z_move_comp")

    def cmd_SET_EXPOSE_CALIBRATION(self,gcmd):
        self.exposure_calibration = gcmd.get_float("VALUE", 1 , above=0.)

    cmd_SET_Z_OFFSET_help = "Sets the z-offset for probing and smart moves"
    def cmd_SET_Z_OFFSET(self,gcmd):
        self.z_offset = gcmd.get_float("OFFSET", 0 , minval=0. )

    cmd_EXPOSE_help = "Exposes a layer for a given time with a given PWM setting"
    def cmd_EXPOSE(self, gcmd):

        if self.exposure_active_flag:
            logging.warning("Exposure already running, please try again later")
            return

        self.last_exposure_power = ((gcmd.get_float("PWM", 0.1 , above=0.) * self.exposure_calibration) * (1-self.uvled_pwm_cutoff_value))+self.uvled_pwm_cutoff_value
        self.last_exposure_time = gcmd.get_float("TIME", 1.0 , above=0.)
        self.last_exposure_pre_delay = gcmd.get_float("PRE_DELAY", 0 )
        self.last_exposure_post_delay = gcmd.get_float("POST_DELAY", 0 )
        self.last_gcmd = gcmd

        self.toolhead = self.printer.lookup_object('toolhead')
        self.ledpwm = self.printer.lookup_object('output_pin LEDPWM')


        if self.enable_powermgmt:
            self.resinheater = self.printer.lookup_object(self.powermanaged_heater_name)
            self.resin_temp_setpoint = self.resinheater.get_temp(self.reactor.monotonic())
            self.resin_temp_setpoint = self.resin_temp_setpoint[1]

            if self.resin_temp_setpoint != 0.0:
                self.resinheater.set_temp(0.0)

        self.exposure_active_flag = True
        self.toolhead.register_lookahead_callback(self.exposure_timing_callback)


    def get_status(self, eventtime):
        return {'last_query': self.last_state,
                'last_z_result': self.last_z_result}


class FssProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()

        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = 'OFF'

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def get_position_endstop(self):
        return 0.


def load_config(config):
    return PrinterFssProbe(config, FssProbeEndstopWrapper(config))
