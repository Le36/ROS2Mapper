#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim
import os
import sys
import termios
import threading
from copy import copy
from typing import Callable

from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Publisher

from .view import View

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

CONTROL_MENU = """
Manual control engaged
---------------------------
Moving around:
        w
   a    s    d
        x
---------------------------

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

m : return to main menu

CTRL-C to quit
"""


class ManualControl(View):
    def __init__(
        self, return_to_menu: Callable[[], None], publisher: Publisher
    ) -> None:
        super().__init__()
        self._return_to_menu = return_to_menu
        self._publisher = publisher

        self._target_lin_vel = 0.0
        self._target_ang_vel = 0.0
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0

    def _print_vels(
        self, target_linear_velocity: float, target_angular_velocity: float
    ):
        """Print the linear and angular velocities of the robot

        Args:
            target_linear_velocity (float): Target linear velocity
            target_angular_velocity (float): Target angular velocity
        """
        os.system("clear")
        print(CONTROL_MENU)
        print(
            f"linear velocity {target_linear_velocity:.2f}\nangular velocity {target_angular_velocity:.1f}"
        )

    def _make_simple_profile(
        self, current: float, target: float, change: float
    ) -> float:
        """Increase or decrease given (linear/angular) velocity

        Args:
            current (float): Current velocity
            target (float): Target velocity
            change (float): Increase/decrease rate

        Returns:
            float: Updated velocity
        """
        if target > current:
            return min(target, current + change)
        if target < current:
            return max(target, current - change)
        return target

    def _bound(self, value: float, min_val: float, max_val: float) -> float:
        """Bound value between min and max

        Args:
            value (float): Value to bound
            min_val (float): Min value
            max_val (float): Max value

        Returns:
            float: Bounded value
        """
        return max(min_val, min(max_val, value))

    def _bound_velocities(self) -> None:
        """Bound velocities"""
        if TURTLEBOT3_MODEL == "burger":
            self._target_lin_vel = self._bound(
                self._target_lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL
            )
            self._target_ang_vel = self._bound(
                self._target_ang_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL
            )
        else:
            self._target_lin_vel = self._bound(
                self._target_lin_vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL
            )
            self._target_ang_vel = self._bound(
                self._target_ang_vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL
            )

    def _handle_io(self) -> None:
        """Handle inputs"""
        settings = termios.tcgetattr(sys.stdin)

        empty_twist = Twist()
        empty_twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        empty_twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        os.system("clear")
        print(CONTROL_MENU)

        while self.running:
            key = self._get_key(settings)
            if key == "w":
                self._target_lin_vel += LIN_VEL_STEP_SIZE
            elif key == "x":
                self._target_lin_vel -= LIN_VEL_STEP_SIZE
            elif key == "a":
                self._target_ang_vel += ANG_VEL_STEP_SIZE
            elif key == "d":
                self._target_ang_vel -= ANG_VEL_STEP_SIZE
            elif key == " " or key == "s":
                self._target_lin_vel = 0.0
                self._target_ang_vel = 0.0
                self._linear_velocity = 0.0
                self._angular_velocity = 0.0
            elif key == "m":
                self._publisher.publish(empty_twist)
                self._return_to_menu()
            elif key == "\x03":
                self.close()
                exit(0)
            elif key:
                print("Input not recognized")

            self._bound_velocities()
            self._linear_velocity = self._make_simple_profile(
                self._linear_velocity,
                self._target_lin_vel,
                (LIN_VEL_STEP_SIZE / 2.0),
            )
            self._angular_velocity = self._make_simple_profile(
                self._angular_velocity,
                self._target_ang_vel,
                (ANG_VEL_STEP_SIZE / 2.0),
            )
            if key in "wxad s":
                self._print_vels(self._target_lin_vel, self._target_ang_vel)

            twist = copy(empty_twist)
            twist.linear.x = self._linear_velocity
            twist.angular.z = self._angular_velocity
            self._publisher.publish(twist)

    def _main(self):
        """Start the manual control view"""
        self.thread = threading.Thread(target=self._handle_io)
        self.thread.start()
