# Force Sensor Probe support for MSLA Printers
#
# Copyright (C) 2022  Pascal Wistinghausen (pascal.wistinghausen@ib-wistinghausen.de)
# Based on previous works by Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins


class PulseOut:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()

        self.output_name = config.get("output_name")

        self.toolhead = None
        self.output = None


        self.report_finished = config.getboolean("report_finished",False)

        self.last_pulse_out_time = 0
        self.last_pulse_out_power = 0
        self.last_pulse_out_pre_delay = 0
        self.last_pulse_out_post_delay = 0
        self.last_gcmd = None

        self.pulse_out_processing_delay = 0.300

        self.pulse_out_active = False

        self.reactor = self.printer.get_reactor()

        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_mux_command("PULSE_OUT", "NAME",
                                        self.name, self.cmd_PULSE_OUT,
                                        desc=self.cmd_PULSE_OUT_help)

        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)


    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.output = self.printer.lookup_object(f'output_pin {self.output_name}')


    def pulse_out_timing_callback(self, print_time):
        reactor_time = self.reactor.monotonic()

        self.reactor.register_callback(self.pulse_out_done_callback, reactor_time + self.last_pulse_out_time + self.pulse_out_processing_delay * 2 + self.last_pulse_out_pre_delay + self.last_pulse_out_post_delay)

        self.output.mcu_pin.set_pwm(print_time + self.pulse_out_processing_delay + self.last_pulse_out_pre_delay, self.last_pulse_out_power)
        self.output.mcu_pin.set_pwm(print_time + self.pulse_out_processing_delay + self.last_pulse_out_pre_delay + self.last_pulse_out_time, 0)

    def pulse_out_done_callback(self, print_time):
        self.pulse_out_active = False
        if self.report_finished:
            self.last_gcmd.respond_raw("Z_move_comp")


    cmd_PULSE_OUT_help = ""
    def cmd_PULSE_OUT(self, gcmd):

        if self.pulse_out_active:
            logging.warning("Exposure already running, please try again later")
            return

        self.last_pulse_out_power = gcmd.get_float("PWM", 0.1, above=0.)
        self.last_pulse_out_time = gcmd.get_float("TIME", 1.0, above=0.)
        self.last_pulse_out_pre_delay = gcmd.get_float("PRE_DELAY", 0)
        self.last_pulse_out_post_delay = gcmd.get_float("POST_DELAY", 0)
        self.last_gcmd = gcmd
        self.pulse_out_active = True
        self.toolhead.register_lookahead_callback(self.pulse_out_timing_callback)


    def get_status(self, eventtime):
        return {
            'none': True
        }

def load_config_prefix(config):
    return PulseOut(config)
