#!/usr/bin/env python

import argparse
import sys
import cv2
import time
import logging

from google.protobuf import duration_pb2

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, block_until_arm_arrives
from bosdyn.choreography.client.choreography import ChoreographyClient

from bosdyn.api import (arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from spot_fsm_control.manipulator import ManipulatorFunctions


class SpotControlInterface(ManipulatorFunctions):

    def __init__(self):
        # super(SpotControlInterface, self).__init__()
        self.hostname = "192.168.80.3"
        self.command_client = None
        self.forward, self.strafe, self.rotate = 0, 0, 0


    def establish_connection(self):
        sdk = bosdyn.client.create_standard_sdk('SpotControlInterface')
        robot = sdk.create_robot(self.hostname)
        bosdyn.client.util.authenticate(robot)
        robot.time_sync.wait_for_sync()
        assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."
        
        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            robot.logger.info("Powering on robot... This may take several seconds.")
            robot.power_on(timeout_sec=20)
            assert robot.is_powered_on(), "Robot power on failed."
            robot.logger.info("Robot powered on.")

            self.image_client = robot.ensure_client(ImageClient.default_service_name)

            # Create a command client to be able to command the robot
            self.command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
            self.manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

            # self.gaze_control()
        

    def stand(self, height):
        cmd = RobotCommandBuilder.stand_command(body_height=height)
        self.command_client.robot_command(cmd)

    def move_command(self, duration=1):
        print("Move:", self.forward, self.strafe, self.rotate)
        cmd = RobotCommandBuilder.velocity_command(self.forward, self.strafe, self.rotate)
        self.command_client.robot_command(cmd, end_time_secs=time.time() + duration) # robot_command_async
        self.forward, self.strafe, self.rotate = 0, 0, 0

    def sit_down(self):
        print("sitting")
        cmd = RobotCommandBuilder.sit_command()
        self.command_client.robot_command(cmd)

    def stop(self):
        print("Stopping")
        cmd = RobotCommandBuilder.stop_command()
        self.command_client.robot_command(cmd)


    def keyboard_movement_control(self):
        wait_time = 20
        while True:
            self.forward, self.strafe, self.rotate = 0, 0, 0

            if cv2.waitKey(wait_time) & 0xFF == ord('p'):
                break
            if cv2.waitKey(wait_time) == ord('a'):
                self.strafe += 0.4
            if cv2.waitKey(wait_time) == ord('d'):
                self.strafe -= 0.4
            if cv2.waitKey(wait_time) == ord('i'): # high
                self.stand(0.5)
            if cv2.waitKey(wait_time) == ord('k'): # low
                self.stand(0.0)
            if cv2.waitKey(wait_time) == ord('q'):
                self.stop()
            if cv2.waitKey(wait_time) == ord('j'):
                self.rotate += 0.2
            if cv2.waitKey(wait_time) == ord('l'):
                self.rotate -= 0.2
            if cv2.waitKey(wait_time) == ord('w'):
                self.forward += 0.4
            if cv2.waitKey(wait_time) == ord('s'):
                self.forward -= 0.4
            if cv2.waitKey(wait_time) == ord("b"):
                self.sit_down()

            if (self.strafe != 0) or (self.forward != 0) or (self.rotate != 0):
                self.move_command()

            cv2.waitKey(1000)

        self.stand(0.4)
        return


if __name__ == "__main__":
    spot = SpotControlInterface()
