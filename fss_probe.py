# Force Sensor Probe support for MSLA Printers
#
# Copyright (C) 2022  Pascal Wistinghausen (pascal.wistinghausen@ib-wistinghausen.de)
# Based on previous works by Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins
from . import manual_probe
import gcode

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

        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")
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

        self.gcode.register_command('ATHENA_MOVE', self.cmd_ATHENA_PROBE_DOWNWARDS,
                                    desc=self.cmd_PROBE_help)

        self.gcode.register_command('QUERY_FSS', self.cmd_QUERY_FSS,
                                    desc=self.cmd_QUERY_FSS_help)

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

    def _probe(self, speed, amount):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")

        phoming = self.printer.lookup_object('homing')
        pos = toolhead.get_position()
        opos = pos[2]
        pos[2] += amount
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
                # in our case this is not an error but desired behavior
                epos = toolhead.get_position()
                epos[2] = amount

            elif "Probe triggered prior to movement" in reason:
                toolhead.move(pos, speed)
                epos = toolhead.get_position()
                epos[2] = amount

            else:
                raise self.printer.command_error(reason)

        return epos[:3]

    def run_probe_upwards(self, gcmd):
        lift_amount = gcmd.get_float("Z", self.lift_amount, minval=0.)
        lift_speed = gcmd.get_float("F", self.lift_speed, above=0.) / 60

        pos = self._probe(lift_speed, lift_amount)

        return pos

    def run_probe_downwards(self, gcmd):
        dip_speed = gcmd.get_float("F", self.lift_speed, above=0.) / 60
        dip_amount = gcmd.get_float("Z", 0, minval=0.)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()

        logging.info("Toolhead Position:",pos[2])

        if dip_amount == 0:
            dip_amount = -1*pos[2]
        else:
            dip_amount = -1 * dip_amount

        pos = self._probe(dip_speed, dip_amount)  # probe to zero

        pos = toolhead.get_position()

        return pos

    cmd_PROBE_help = "Probe Z-height at current XY position"

    def cmd_ATHENA_MOVE(self, gcmd):
        cmd_G1(gcmd)
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
        self.last_z_result = pos[2]

    cmd_QUERY_FSS_help = "Return the status of the z-probe"

    def cmd_QUERY_FSS(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("fss input: %s" % (["open", "TRIGGERED"][not not res],))

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
