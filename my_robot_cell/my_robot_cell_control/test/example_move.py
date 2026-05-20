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
import time
import unittest

import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.node import Node as ROSNode

from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import GetRobotMode


UR_TYPE = "ur20"
TF_PREFIX = f"{UR_TYPE}_"
ROBOT_IP = "192.168.56.101"

TIMEOUT_WAIT_SERVICE = 10
# When the test runs together with downloading the URSim docker image, the dashboard server can
# take quite a while to come up.
TIMEOUT_WAIT_SERVICE_INITIAL = 120
TIMEOUT_EXAMPLE_MOVE = 120


@pytest.mark.launch_test
def generate_test_description():
    ursim = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_client_library"),
                    "lib",
                    "ur_client_library",
                    "start_ursim.sh",
                ]
            ),
            "-m",
            UR_TYPE,
        ],
        name="start_ursim",
        output="screen",
    )

    wait_dashboard_server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("ur_robot_driver"), "bin", "wait_dashboard_server.sh"]
            )
        ],
        name="wait_dashboard_server",
        output="screen",
    )

    # Use the tutorial's launch file unchanged. Running it in headless mode lets the test drive
    # the robot via the dashboard client without manually loading the external_control program.
    tutorial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_cell_control"),
                    "launch",
                    "start_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": UR_TYPE,
            "robot_ip": ROBOT_IP,
            "launch_rviz": "false",
            "headless_mode": "true",
        }.items(),
    )

    driver_starter = RegisterEventHandler(
        OnProcessExit(target_action=wait_dashboard_server, on_exit=tutorial_launch)
    )

    return LaunchDescription([ReadyToTest(), wait_dashboard_server, ursim, driver_starter])


def _wait_for_service(node, srv_name, srv_type, timeout):
    client = node.create_client(srv_type, srv_name)
    logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)
    if not client.wait_for_service(timeout):
        raise RuntimeError(f"Could not reach service '{srv_name}' within {timeout}s")
    return client


def _call_service(node, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT_WAIT_SERVICE)
    if future.result() is None:
        raise RuntimeError(f"Error while calling service '{client.srv_name}': {future.exception()}")
    return future.result()


class ExampleMoveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = ROSNode("custom_workcell_example_move_test")

        cls.power_on = _wait_for_service(
            cls.node, "/dashboard_client/power_on", Trigger, TIMEOUT_WAIT_SERVICE_INITIAL
        )
        cls.power_off = _wait_for_service(
            cls.node, "/dashboard_client/power_off", Trigger, TIMEOUT_WAIT_SERVICE
        )
        cls.brake_release = _wait_for_service(
            cls.node, "/dashboard_client/brake_release", Trigger, TIMEOUT_WAIT_SERVICE
        )
        cls.unlock_protective_stop = _wait_for_service(
            cls.node,
            "/dashboard_client/unlock_protective_stop",
            Trigger,
            TIMEOUT_WAIT_SERVICE,
        )
        cls.stop = _wait_for_service(
            cls.node, "/dashboard_client/stop", Trigger, TIMEOUT_WAIT_SERVICE
        )
        cls.get_robot_mode = _wait_for_service(
            cls.node, "/dashboard_client/get_robot_mode", GetRobotMode, TIMEOUT_WAIT_SERVICE
        )
        cls.resend_robot_program = _wait_for_service(
            cls.node,
            "/io_and_status_controller/resend_robot_program",
            Trigger,
            TIMEOUT_WAIT_SERVICE_INITIAL,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _check_success(self, response):
        self.assertTrue(response.success, msg=getattr(response, "message", ""))

    def _start_robot_once(self):
        """Power the robot on, release the brakes and wait until it reports RUNNING."""
        self._check_success(_call_service(self.node, self.power_off, Trigger.Request()))
        self._check_success(_call_service(self.node, self.power_on, Trigger.Request()))
        self._check_success(_call_service(self.node, self.brake_release, Trigger.Request()))
        # unlock_protective_stop is not guaranteed to succeed if the robot is not in a
        # protective stop, so we don't assert success here.
        _call_service(self.node, self.unlock_protective_stop, Trigger.Request())

        start_time = time.time()
        while time.time() - start_time < TIMEOUT_WAIT_SERVICE_INITIAL:
            robot_mode = _call_service(self.node, self.get_robot_mode, GetRobotMode.Request())
            self._check_success(robot_mode)
            if robot_mode.robot_mode.mode == RobotMode.RUNNING:
                _call_service(self.node, self.stop, Trigger.Request())
                return
            time.sleep(0.5)
        raise AssertionError(
            f"Robot did not reach RUNNING mode within {TIMEOUT_WAIT_SERVICE_INITIAL}s "
            f"(last mode: {robot_mode.robot_mode.mode})"
        )

    def test_example_move(self):
        # Start the robot exactly once before invoking example_move.
        self._start_robot_once()
        time.sleep(1)
        self._check_success(_call_service(self.node, self.resend_robot_program, Trigger.Request()))

        cmd = [
            "ros2",
            "run",
            "ur_robot_driver",
            "example_move.py",
            "--ros-args",
            "-p",
            f"tf_prefix:={TF_PREFIX}",
        ]
        logging.info("Running example_move: %s", " ".join(cmd))
        subprocess.check_call(cmd, timeout=TIMEOUT_EXAMPLE_MOVE)
