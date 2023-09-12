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
from bosdyn.util import seconds_to_duration

from spot_fsm_control.arm_impedance_control_helpers import (apply_force_at_current_position,
                                           get_impedance_mobility_params, get_root_T_ground_body)


class SpotControlInterface(ManipulatorFunctions):

    def __init__(self, direct_control_frequency):
        # super(SpotControlInterface, self).__init__()
        self.hostname = "192.168.80.3"
        self.command_client = None
        self.forward, self.strafe, self.rotate = 0, 0, 0
        self.prev_close_or_open = "close"
        self.direct_control_trajectory_list = []
        self.current_state_direct_control = False
        self.direct_control_frequency = direct_control_frequency


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

    def stop(self):
        cmd = RobotCommandBuilder.stop_command()
        self.command_client.robot_command(cmd)
        cmd = RobotCommandBuilder.arm_stow_command()
        self.command_client.robot_command(cmd)

    def stand(self, height):
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height)
        self.command_client.robot_command(cmd)

    def move_command(self, duration=1):
        print("Move:", self.forward, self.strafe, self.rotate)
        cmd = RobotCommandBuilder.synchro_velocity_command(self.forward, self.strafe, self.rotate)
        self.command_client.robot_command(cmd, end_time_secs=time.time() + duration) # robot_command_async
        self.forward, self.strafe, self.rotate = 0, 0, 0

    def sit_down(self):
        print("sitting")
        cmd = RobotCommandBuilder.synchro_sit_command()
        self.command_client.robot_command(cmd)

    def stop(self):
        print("Stopping")
        cmd = RobotCommandBuilder.stop_command()
        self.command_client.robot_command(cmd)

    def gripper(self, close_or_open):
        if close_or_open == "open":
            cmd = RobotCommandBuilder.claw_gripper_open_command()
            self.command_client.robot_command(cmd)
        elif close_or_open == "close":
            # rubber tomato 0.375
            cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(0.375)
            self.command_client.robot_command(cmd)
        self.prev_close_or_open = close_or_open
            
    def cartesian_hand_position(self, pos, quat, time=1):
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=pos[0], y=pos[1], z=pos[2])
        flat_body_Q_hand = geometry_pb2.Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])

        flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                rotation=flat_body_Q_hand)

        robot_state = self.robot_state_client.get_robot_state()
        target_frame = "hand"
        source_frame = "flat_body"
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         source_frame, target_frame)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

        # duration in seconds
        seconds = time

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, source_frame, seconds)

        # Send the request
        cmd_id = self.command_client.robot_command(arm_command)

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives(self.command_client, cmd_id)
        
    def move_to_cartesian_pose_rt_task(self, task_T_desired, root_T_task, wr1_T_tool):
        robot_cmd = RobotCommandBuilder.synchro_stand_command(params=get_impedance_mobility_params())
        arm_cart_cmd = robot_cmd.synchronized_command.arm_command.arm_cartesian_command

        # Set up our root frame, task frame, and tool frame.
        arm_cart_cmd.root_frame_name = GRAV_ALIGNED_BODY_FRAME_NAME
        arm_cart_cmd.root_tform_task.CopyFrom(root_T_task.to_proto())
        arm_cart_cmd.wrist_tform_tool.CopyFrom(wr1_T_tool.to_proto())

        # Do a single point goto to a desired pose in the task frame.
        cartesian_traj = arm_cart_cmd.pose_trajectory_in_task
        traj_pt = cartesian_traj.points.add()
        traj_pt.time_since_reference.CopyFrom(seconds_to_duration(1.0))
        traj_pt.pose.CopyFrom(task_T_desired.to_proto())

        # Execute the Cartesian command.
        cmd_id = self.command_client.robot_command(robot_cmd)
        
        # block_until_arm_arrives(self.command_client, cmd_id, 1/self.direct_control_frequency)
        
    def stop_direct_control(self):
        self.current_state_direct_control = False
        print("Stopping Direct Control")
                
    def ready_or_stow_arm(self, stow=False):
        if stow:
            cmd = RobotCommandBuilder.arm_stow_command()
        else:
            cmd = RobotCommandBuilder.arm_ready_command()
        cmd_id = self.command_client.robot_command(cmd) 
        block_until_arm_arrives(self.command_client, cmd_id)

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
