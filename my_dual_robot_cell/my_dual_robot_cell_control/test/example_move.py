#!/usr/bin/env python
# Copyright 2026, Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
import subprocess
import unittest

import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest


ROBOTS = {
    "alice": {
        "tf_prefix": "alice_",
        "controller_name": "alice_joint_trajectory_controller",
    },
    "bob": {
        "tf_prefix": "bob_",
        "controller_name": "bob_joint_trajectory_controller",
    },
}

TIMEOUT_EXAMPLE_MOVE = 120


@pytest.mark.launch_test
def generate_test_description():
    # Mock hardware lets us exercise the tutorial's launch file and controllers without bringing
    # up two URSim instances, dashboard clients or the external_control program.
    tutorial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "launch",
                    "start_robots.launch.py",
                ]
            )
        ),
        launch_arguments={
            "alice_use_mock_hardware": "true",
            "bob_use_mock_hardware": "true",
            "launch_rviz": "false",
        }.items(),
    )

    return LaunchDescription([ReadyToTest(), tutorial_launch])


class DualArmExampleMoveTest(unittest.TestCase):
    def _run_example_move(self, name):
        cfg = ROBOTS[name]
        cmd = [
            "ros2",
            "run",
            "ur_robot_driver",
            "example_move.py",
            "--ros-args",
            "-p",
            f"tf_prefix:={cfg['tf_prefix']}",
            "-p",
            f"controller_name:={cfg['controller_name']}",
        ]
        logging.info("Running example_move for '%s': %s", name, " ".join(cmd))
        subprocess.check_call(cmd, timeout=TIMEOUT_EXAMPLE_MOVE)

    def test_example_move_both_arms(self):
        # example_move.py waits for the controller's action server itself, so we can simply run
        # it once per arm.
        for name in ROBOTS:
            self._run_example_move(name)
