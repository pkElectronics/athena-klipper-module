# Force Sensor Probe support for MSLA Printers
#
# Copyright (C) 2022  Pascal Wistinghausen (pascal.wistinghausen@ib-wistinghausen.de)
# Based on previous works by Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
from time import sleep
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
        self.smart_move_segment_resolution = config.getint("smart_move_segment_resolution", 5, minval=1)
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

        self.kinematic_mode = "minimal"

        self.expose_processing_delay = 0.300

        self.exposure_active_flag = False
        self.uvled_pwm_cutoff_value = 0.25

        self.z_offset = 0.0

        self.x_res = 15120
        self.y_res = 6230
        self.x_px = 19
        self.y_px = 14
        self.total_screen_area = (self.x_res * self.x_px) * (self.y_res * self.y_px)

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

        self.gcode.register_command('ATHENA_SET_KINEMATIC_MODE_MINIMAL', self.cmd_ATHENA_SET_KINEMATIC_MODE_MINIMAL)

        self.gcode.register_command('ATHENA_SET_KINEMATIC_MODE_FULL', self.cmd_ATHENA_SET_KINEMATIC_MODE_FULL)
        self.gcode.register_command('ATHENA_SET_KINEMATIC_MODE', self.cmd_ATHENA_SET_KINEMATIC_MODE)



        self.gcode.register_command('ATHENA_SET_MINIMUM_LIFT_DISTANCE', self.cmd_ATHENA_SET_MINIMUM_LIFT_DISTANCE)

        self.gcode.register_command('ATHENA_SET_FULL_LIFT_SPEED', self.cmd_ATHENA_SET_FULL_LIFT_SPEED)

        self.gcode.register_command('ATHENA_DIP', self.cmd_ATHENA_DIP)
        self.gcode.register_command('ATHENA_PEEL', self.cmd_ATHENA_PEEL)


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
        p = position.copy()
        p[2] += self.z_offset
        toolhead.move(p,speed)

    def _reset_accel_decel(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_kinematics().reset_accel_decel()

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
        opos = pos[2] + self.z_offset
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
        lift_segment_distance = 0.2
        target_accel = 1000.
        base_accel = 0.1

        self._reset_accel_decel()

        toolhead = self.printer.lookup_object('toolhead')

        position = self._get_position()

        stage1_position = position.copy()
        stage1_position[2] += stage1_lift_distance

        kinematics = toolhead.get_kinematics()

        saved_accel_decel = kinematics.get_accel_decel()
        stage1_accel_decel = saved_accel_decel.copy()
        stage1_accel_decel["peel_accel"] = base_accel
        stage1_accel_decel["peel_decel"] = target_accel
        kinematics.set_accel_decel(stage1_accel_decel)


        segments = int(self.min_lift_distance / lift_segment_distance)
        for i in range(1,segments):
            pos = position.copy()
            pos[2] += i*lift_segment_distance
            acc = saved_accel_decel.copy()
            acc["peel_accel"] = min(target_accel,base_accel*2**i)
            kinematics.set_accel_decel(acc)
            self._move(pos,lift_speed)


        stage1_accel_decel["peel_accel"] = 1000
        kinematics.set_accel_decel(stage1_accel_decel)

        print_time = toolhead.get_last_move_time()

        if not self.mcu_probe.query_endstop(print_time):
            pos = self._probe(lift_speed, lift_amount - stage1_lift_distance)
        else:
            pos = [0.0,0.0,self.min_lift_distance]

        kinematics.set_accel_decel(saved_accel_decel)

        if self.kinematic_mode == "minimal":
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

        elif self.kinematic_mode == "full":
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

        self._reset_accel_decel()

        return pos


    def _smart_peel_compute_lift_accel(self,base_accel, target_accel, segment_idx, total_segments):

        # normalize to 0..1
        t = segment_idx / total_segments

        # clamp
        if t <= 0.0:
            ratio = 0.0
        elif t >= 1.0:
            ratio = 1.0
        else:
            # smoothstep
            ratio = t * t * (3.0 - 2.0 * t)

        acc = ((target_accel - base_accel) * ratio) + base_accel

        return max(round(acc,1),base_accel)

    def smart_peel(self, gcmd):
        lift_total = gcmd.get_float("LIFT_TOTAL", above=0.)
        lift_speed = gcmd.get_float("SPEED", self.lift_speed, above=0.)

        total_surface_area_mm2 = gcmd.get_float("TOTAL_SURFACEAREA", minval=0.) #from print data
        largest_surface_area_mm2 = gcmd.get_float("LARGEST_SURFACEAREA", minval=0.) #from print data
        modulus_gpa = gcmd.get_float("MODULUS", above=0.) #from resin profile
        viscosity_cps = gcmd.get_float("VISCOSITY", above=0.) #from resin profile

        if largest_surface_area_mm2 == 0.0:
            largest_surface_area_mm2 = self.buildplate_area * 0.2

        stage1_max_speed = lift_speed / 2
        stage1_min_speed = lift_speed / 4
        stage2_max_speed = self.full_lift_speed * 60
        stage2_min_speed = lift_speed / 2

        effective_resin_level = min(self.last_resin_level,viscosity_cps / 250.0) #this is kinda experimental

        #calculations are performed in mm/min

        pos = self._get_position()
        layer_position = pos[2]

        stage1_distance = min(lift_total - 1, max(1, round(lift_total / pow(3 * modulus_gpa, 2 / 3), 1)))
        stage2_distance = lift_total - stage1_distance

        logging.warning(f"Smart Peel first calc run: S1D: {stage1_distance} | S2D: {stage2_distance}")

        if layer_position < effective_resin_level:
            speed = lift_speed/2
            #stage1_distance = max(stage1_distance, round(actual_lift_distance/2, 2))

        else:
            areaRatio = largest_surface_area_mm2 / self.buildplate_area
            areaFactor = pow(areaRatio, 1 / 4)
            minSpeed = max(stage1_max_speed, lift_speed * (modulus_gpa / 6 + 1/2))
            stage2_distance = round((1 + stage2_distance * areaFactor),1)
            speed = round((minSpeed + (lift_speed - minSpeed) * (1-areaFactor)) , 2)


        #stage1Speed = max(stage1_min_speed, round(speed * 0.15 * (1 / modulus_gpa),2)) #matteos version
        stage1Speed = max(stage1_min_speed, round(speed ** 2 / lift_speed * 1 / 2, 2)) #jacobis version (corrected on 6.3.)

        stage1Speed = min(stage1_max_speed,stage1Speed)
        stage2Speed = max(stage2_min_speed, speed)
        stage2Speed = min(stage2_max_speed, stage2Speed)

        actual_lift_distance = stage1_distance + stage2_distance # recompute due to likely changes in previous code

        #convert speeds to mm/s for klipper:
        stage1Speed = round(stage1Speed / 60, 2)
        stage2Speed = round(stage2Speed / 60, 2)

        lift_segment_distance_um = self.smart_move_segment_resolution
        target_accel = 1000.
        base_accel = 0.1

        toolhead = self.printer.lookup_object('toolhead')
        position = self._get_position()

        end_z = position[2] + stage1_distance + stage2_distance
        end_position = position.copy()
        end_position[2]= end_z

        kinematics = toolhead.get_kinematics()
        self._reset_accel_decel()
        saved_accel_decel = kinematics.get_accel_decel()
        stage1_accel_decel = saved_accel_decel.copy()
        stage2_accel_decel = saved_accel_decel.copy()
        stage1_accel_decel["peel_accel"] = base_accel
        stage1_accel_decel["peel_decel"] = base_accel
        kinematics.set_accel_decel(stage1_accel_decel)

        stage1_distance_um = round(stage1_distance * 1000.0)

        segments = int(stage1_distance_um / lift_segment_distance_um)

        remainder = stage1_distance_um - (segments * lift_segment_distance_um)

        for i in range(0, segments):
            pos = position.copy()
            pos[2] += (i+1) * lift_segment_distance_um / 1000.
            acc = saved_accel_decel.copy()
            acc["peel_accel"] = acc["peel_decel"] = self._smart_peel_compute_lift_accel(base_accel,target_accel,i,segments)
            kinematics.set_accel_decel(acc)
            self._move(pos, stage1Speed)
            eventtime = self.reactor.monotonic()
            eventtime = self.reactor.pause(eventtime + 0.005) # this is a hack

        if remainder > 0:
            logging.warning(f"Remainder Move: {remainder}um")
            pos = position.copy()
            pos[2] += (segments * lift_segment_distance_um / 1000.) + ( remainder / 1000.)

        stage1_accel_decel["peel_accel"] = stage1_accel_decel["peel_decel"] = 1000
        kinematics.set_accel_decel(stage1_accel_decel)

        pd_trigger_position = 0.0

        waittime = 0.0

        if position[2] > self.last_resin_level:

            while len(toolhead.lookahead.queue) > 10 and waittime < 10.:
                eventtime = self.reactor.monotonic()
                eventtime = self.reactor.pause(eventtime + 0.10)
                waittime += 0.1


            if waittime > 10.:
                logging.warning("PD State timeout")
            self._reset_accel_decel()

            print_time = toolhead.get_last_move_time()

            if not self.mcu_probe.query_endstop(print_time):
                logging.warning(f"Smart Peel - Above Resin Level - PeelDetection Not Triggered")
                pos = self._probe(stage2Speed, stage2_distance)
                if pos[2] != stage2_distance:
                    logging.warning(f"Smart Peel - PeelDetection Triggered - Doing final move")
                    pd_trigger_position = pos[2] + stage1_distance
                    stage2_accel_decel["peel_accel"] = self._smart_peel_compute_lift_accel(base_accel,target_accel,segments,segments)
                    kinematics.set_accel_decel(stage2_accel_decel)
                    self._move(end_position, self.full_lift_speed)
                else:
                    pd_trigger_position = actual_lift_distance


                pos = [0.0, 0.0, pd_trigger_position]


            else:
                logging.warning(f"Smart Peel - Above Resin Level - PeelDetection Triggered in first stage")
                pd_trigger_position = stage1_distance
                pos = [0.0, 0.0, pd_trigger_position]
                stage2_accel_decel["peel_accel"] = self._smart_peel_compute_lift_accel(base_accel, target_accel,segments,segments)
                kinematics.set_accel_decel(stage2_accel_decel)
                self._move(end_position, self.full_lift_speed)

        else:
            logging.warning(f"Smart Peel - Below Resin Level - PeelDetection Disabled")
            pos = [0.0, 0.0, actual_lift_distance]
            self._move(end_position, stage2Speed)

        toolhead.wait_moves()

        kinematics.set_accel_decel(saved_accel_decel)
        #logging.warning(f"Smart Peel - Commanded Lift: {lift_total} | Stage 1 Lift: {stage1_distance} | Stage 1 Speed: {stage1Speed} | Stage 2 Lift: {stage2_distance} | Stage 2 Speed: {stage2Speed}")

        logstr = f"Smart Peel Move Stats - LP:{layer_position} | CL:{lift_total} | AL:{actual_lift_distance} | S1L:{stage1_distance} | S2L:{stage2_distance} | S1S:{round(stage1Speed*60.0,2)} | S2S:{round(stage2Speed*60.0,2)} | PDT:{round(pd_trigger_position,2)} | Peel Detection: "
        if pd_trigger_position == 0.0:
            logstr += "Disabled"
        elif pd_trigger_position == stage1_distance:
            logstr += "During Stage 1"
        elif pd_trigger_position == actual_lift_distance:
            logstr+= "Not Triggered"
        else:
            logstr += f"During Stage 2 at {pd_trigger_position}mm ({round((pd_trigger_position/actual_lift_distance)*100)}% of total move"
        logging.warning(logstr)

        self._reset_accel_decel()

        return pos



    def smart_dip(self, gcmd):
        #this version is an approximation of a constant pressure velocity profile using a similar scheme as a g2 command
        self._reset_accel_decel()
        mPa_to_gfmm2 = .0000102
        modulus_to_pressure = 100000.0 #This is pure guesswork
        viscosity_cps = gcmd.get_float("VISCOSITY", above=0.) #from resin profile
        resin_level_mm = self.last_resin_level
        surface_area_mm2 = gcmd.get_float("SURFACEAREA", minval=0.) #from print data
        buildplate_area_mm2 = self.buildplate_area
        layerheight_mm = gcmd.get_float("LAYERHEIGHT", above=0.) # from slice data
        modulus_gpa = gcmd.get_float("PRESSURE", above=0.) #from resin profile
        pressure_mpa = modulus_gpa * modulus_to_pressure
        pressure_gfmm2 = pressure_mpa * mPa_to_gfmm2

        target_position = gcmd.get_float("TARGET", minval=0.)
        target_speed = gcmd.get_float("SPEED", minval=0.)

        if surface_area_mm2 > 40000 or surface_area_mm2 == 0:
            surface_area_mm2 = self.total_screen_area*0.2

        # constant factors for generating velocity profile
        resolution = self.smart_move_segment_resolution # similar to g2 gcode

        vmin = 0.05 # minimum velocity 0.3mm/min
        vmax = target_speed / 60   # maximum velocity 600mm/min
        max_force = 20000 #maximum force a retract move will try to achieve
        viscosity_coefficient = 60   #constant for movements outside of squeezing flow regime
        e_plate_area = buildplate_area_mm2 * 0.75 # the circular eqiuvalent area which produces the same constant pressure curve
        pressure_mpa_maxforce = (max_force/e_plate_area)/mPa_to_gfmm2    #pressure on build plate corresponding to maximum force

        pos = self._get_position()
        dip_amount = pos[2] - target_position

        dip_first_stage = round(dip_amount * 0.8,2) #documentation only, not used for compute
        dip_second_stage = round(dip_amount * 0.2,2)

        first_stage_target_position = [0. , 0. , 0. , 0.]

        first_stage_target_position[2] =  target_position + dip_second_stage
        self._move(first_stage_target_position,vmax)


        d_d = max(0.1,(0.218*((surface_area_mm2*pressure_gfmm2)**0.821)) / 1000)    # maximum arm deflection from retract force on layer area
        d_d2 = max(0.1, (0.218 * (max_force ** 0.821)) / 1000) # maximum arm deflection due to force on build plate
        logging.info(f"Smart Dip Move Stats - DA:{dip_amount} | TZ:{target_position} | S1D:{dip_first_stage} | S1T:{first_stage_target_position[2]} | S2D:{dip_second_stage}")

        dip_second_stage_um = round(dip_second_stage * 1000.0)
        segments = round(dip_second_stage_um / resolution)

        remainder = dip_second_stage_um - (segments * resolution)

        if target_position < resin_level_mm:
            for i in range(1, segments + 1):
                step_pos = [0. , 0. , 0. , 0.]
                step_pos[2] =  first_stage_target_position[2] - (i * resolution) / 1000.
                di = dip_second_stage + layerheight_mm - (i*resolution)/ 1000.
                # velocity = minimum of velocity for maximum part pressure or velocity corresponding with maximum force
                v1 = (3*(pressure_mpa*math.pi*2*(d_d+di)**3)/(3*viscosity_cps*surface_area_mm2))*(1 + (viscosity_coefficient*math.atan(((di)+d_d)/(math.sqrt(surface_area_mm2/math.pi)))))
                v2 = ((pressure_mpa_maxforce*math.pi*2*(d_d2+di)**3)/(3*viscosity_cps*e_plate_area))*(1 + (viscosity_coefficient*math.atan(((di)+d_d2)/(math.sqrt(e_plate_area/math.pi)))))
                v3 = min(v1,v2)
                velocity = min(max(v3,vmin),vmax)
                self._move(step_pos,velocity)

        else:
            for i in range(1, segments + 1):
                step_pos = [0. , 0. , 0. , 0.]
                step_pos[2] =  first_stage_target_position[2] - (i * resolution)/ 1000.
                di = dip_second_stage + layerheight_mm - (i*resolution)/ 1000.
                v1 = (3*(pressure_mpa*math.pi*2*(d_d+di)**3)/(3*viscosity_cps*surface_area_mm2))*(1 + (viscosity_coefficient*math.atan(((di)+d_d)/(math.sqrt(surface_area_mm2/math.pi)))))
                velocity = min(max(v1,vmin),vmax)
                self._move(step_pos,velocity)

        if remainder > 0:
            logging.warning(f"Remainder Move: {remainder}um")
            step_pos = [0., 0., target_position, 0.]
            self._move(step_pos,vmin)

        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        pos = self._get_position()
        self._reset_accel_decel()

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

    def cmd_ATHENA_DIP(self,gcmd):
        if self.kinematic_mode != "smart":
            target_position = gcmd.get_float("TARGET", minval=0.)
            target_speed = gcmd.get_float("SPEED", minval=0.) / 60.0
            pos = self._get_position()
            pos[2] = target_position
            self._move(pos,target_speed)
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()

        else:
            self.smart_dip(gcmd)

        gcmd.respond_raw("Z_move_comp")

    def cmd_ATHENA_PEEL(self,gcmd):
        if self.kinematic_mode != "smart":
            pos = self.run_probe_upwards(gcmd)
        else:
            pos = self.smart_peel(gcmd)

        gcmd.respond_raw("Z_move_comp")
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]


    def cmd_ATHENA_SMART_DIP(self,gcmd):
        self.smart_dip(gcmd)
        gcmd.respond_raw("Z_move_comp")

    def cmd_ATHENA_SMART_PEEL(self,gcmd):
        pos = self.smart_peel(gcmd)
        gcmd.respond_raw("Z_move_comp")
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]


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

    def cmd_ATHENA_SET_KINEMATIC_MODE_MINIMAL(self, gcmd):
        self.kinematic_mode= "minimal"

    def cmd_ATHENA_SET_KINEMATIC_MODE_FULL(self, gcmd):
        self.kinematic_mode= "full"

    def cmd_ATHENA_SET_KINEMATIC_MODE(self, gcmd):
        mode = gcmd.get_int("MODE")
        if mode == 0:
            self.kinematic_mode = "minimal"
        elif mode == 1:
            self.kinematic_mode = "full"
        elif mode == 2:
            self.kinematic_mode = "smart"
        else:
            self.kinematic_mode = "full"

    def cmd_ATHENA_SET_MINIMUM_LIFT_DISTANCE(self, gcmd):
        self.min_lift_distance = gcmd.get_float("VALUE", self.lift_amount, minval=0.)

    def cmd_ATHENA_SET_FULL_LIFT_SPEED(self, gcmd):
        self.full_lift_speed = gcmd.get_float("VALUE", self.lift_speed * 60, minval=0.) / 60

    def cmd_ATHENA_UPDATE_SCREEN_PARAMS(self,gcmd):
        self.x_res = gcmd.get_int("XRES")
        self.y_res = gcmd.get_int("YRES")
        self.x_px = gcmd.get_int("XPX")
        self.y_px = gcmd.get_int("YPX")
        self.total_screen_area = (self.x_res * self.x_px) * (self.y_res * self.y_px)


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
        self.z_offset = gcmd.get_float("OFFSET", 0.0 , minval=0. )
        logging.info(f"Update Z-Offset to: {self.z_offset}")

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

        toolhead = self.printer.lookup_object('toolhead')
        self.ledpwm = self.printer.lookup_object('output_pin LEDPWM')


        if self.enable_powermgmt:
            self.resinheater = self.printer.lookup_object(self.powermanaged_heater_name)
            self.resin_temp_setpoint = self.resinheater.get_temp(self.reactor.monotonic())
            self.resin_temp_setpoint = self.resin_temp_setpoint[1]

            if self.resin_temp_setpoint != 0.0:
                self.resinheater.set_temp(0.0)

        self.exposure_active_flag = True
        toolhead.register_lookahead_callback(self.exposure_timing_callback)


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
