# Code for handling the kinematics of a single z-axis mSLA Printer
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# Based on previous work of Pascal Wistinghausen of Concepts3D <pascal@concepts3d.ca>
# With improvements by Ada Phillips
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
import math


class ZonlyKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup z axis rail
        self.z_rail = stepper.LookupMultiRail(config.getsection('stepper_z'))

        self.z_rail.setup_itersolve('cartesian_stepper_alloc', 'z'.encode())
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)

        self.peel_accel = config.getfloat('peel_accel', max_accel,
                                          above=0., maxval=max_accel)

        self.dip_accel = config.getfloat('dip_accel', max_accel,
                                         above=0., maxval=max_accel)

        self.peel_decel = config.getfloat('peel_decel', max_accel,
                                          above=0., maxval=max_accel)

        self.dip_decel = config.getfloat('dip_decel', max_accel,
                                         above=0., maxval=max_accel)

        self.homing_accel_decel = config.getfloat('homing_accel_decel', max_accel / 10,
                                                  above=0., maxval=max_accel)

        self.limit = (1.0, -1.0)
        z_range = self.z_rail.get_range()

        self.axes_min = toolhead.Coord(0, 0, z_range[0], e=0.)
        self.axes_max = toolhead.Coord(0, 0, z_range[1], e=0.)

    def get_steppers(self):
        return self.z_rail.get_steppers()

    def calc_position(self, stepper_positions):
        return [0, 0, stepper_positions[self.z_rail.get_name()]]

    def set_position(self, newpos, homing_axes):
        self.z_rail.set_position(newpos)
        if 2 in homing_axes:
            self.limit = self.z_rail.get_range()

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit = (1.0, -1.0)

    def _home_z_axis(self, homing_state):
        # Determine movement
        position_min, position_max = self.z_rail.get_range()
        hi = self.z_rail.get_homing_info()
        forcepos = hi.position_endstop
        if hi.positive_dir:
            forcepos -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos += 1.5 * (position_max - hi.position_endstop)
        # Perform homing

        self.save_peel_accel = self.peel_accel
        self.save_dip_accel = self.dip_accel
        self.save_peel_decel = self.peel_decel
        self.save_dip_decel = self.dip_decel

        self.peel_accel = self.homing_accel_decel
        self.dip_accel = self.homing_accel_decel
        self.peel_decel = self.homing_accel_decel
        self.dip_decel = self.homing_accel_decel

        homing_state.home_rails(
            [self.z_rail],
            [None, None, forcepos, None],
            [None, None, hi.position_endstop, None]
        )

        self.peel_accel = self.save_peel_accel
        self.dip_accel = self.save_dip_accel
        self.peel_decel = self.save_peel_decel
        self.dip_decel = self.save_dip_decel

    def home(self, homing_state):
        # Only z axis homing is respected
        if 2 in homing_state.get_axes():
            self._home_z_axis(homing_state)

    def _motor_off(self, print_time):
        self.limit = (1.0, -1.0)

    def _check_endstops(self, move):
        end_pos = move.end_pos[2]
        if (move.axes_d[2] and
                (end_pos < self.limit[0] or end_pos > self.limit[1])):
            if self.limit[0] > self.limit[1]:
                raise move.move_error("Must home axis first")
            raise move.move_error()

    def check_move(self, move):
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])

        move_accel = self.max_z_accel
        move_decel = self.max_z_accel

        if move.start_pos[2] > move.end_pos[2]:  # downwards move
            move_accel = self.dip_accel
            move_decel = self.dip_decel
        else:
            move_accel = self.peel_accel
            move_decel = self.peel_decel

        z_small_move_ratio = min((abs(move.axes_d[2]) / 2), 1)

        if z_small_move_ratio < 1:
            move_accel = min(move_accel,move_decel)
            move_decel = move_accel
        
        reachable_z_velocity = self.max_z_velocity

        for i in range(1,int(self.max_z_velocity), 1):
            test_v = float(i)
            accel_t = test_v/move_accel
            decel_t = test_v/move_decel
            accel_d = 0.5*move_accel*accel_t**2
            decel_d = 0.5*move_decel*decel_t**2

            if (accel_d+decel_d) < abs(move.axes_d[2]):
                reachable_z_velocity = test_v
            else:
                break

        logging.info("Kinematics output reachable_velocity: %f accel: %f decel: %f ratio: %f" % (reachable_z_velocity, move_accel, move_decel, z_small_move_ratio))

        move.limit_speed(reachable_z_velocity, move_accel * z_ratio, move_decel * z_ratio)

    def get_status(self, eventtime):
        return {
            'homed_axes': "z" if self.limit[0] < self.limit[1] else "",
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }


def load_kinematics(toolhead, config):
    return ZonlyKinematics(toolhead, config)
