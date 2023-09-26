#!/usr/bin/env python

import time
import sys
sys.path.append("./hagrid/")
sys.path.append("./src/")

import math
import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
import ast
import numpy as np
import logging
# mp_drawing = mp.solutions.drawing_utils
# mp_drawing_styles = mp.solutions.drawing_styles

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, block_until_arm_arrives
from bosdyn.choreography.client.choreography import ChoreographyClient

from bosdyn.api import (image_pb2, arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import Quat, SE3Pose

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from spot_fsm_control.spot_control_interface import SpotControlInterface
from spot_fsm_control.finite_state_machine import SpotStateMachine
from spot_fsm_control.arm_impedance_control_helpers import get_root_T_ground_body

logging.basicConfig(format="[LINE:%(lineno)d] %(levelname)-8s [%(asctime)s]  %(message)s", level=logging.INFO)

DIRECT_CONTROL_FREQUENCY = 15 #Hz Max 60,


def try_state_send(state_machine, action):
    print("Current state:", state_machine.current_state)
    try:
        state_machine.send(action)
    except Exception as e:
        print(e)
        try:
            state_machine.send("stop_action")
            print("Stop action first")
            state_machine.send(action)
        except Exception as e:
            print(e)
            try:
                state_machine.send("stand_up")
                print("Stand up first")
                state_machine.send(action)
            except Exception as e:
                print(f"{action} not possible")
                print(e)
                ## Do some handling or more feedback to user


class FsmNode:

    def __init__(self, robot: SpotControlInterface):
        self.robot = robot
        self.sm = SpotStateMachine(robot=robot)
        self.arm_pos_init = [0, 0, 0]
        self.arm_ori_init = [1, 0, 0, 0]
        zero_pose = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        zero_trajectory_pose = trajectory_pb2.SE3TrajectoryPoint(pose=zero_pose.to_proto())
        self.direct_control_trajectory_list = [zero_trajectory_pose]
        
        
        # Pick a task frame that is beneath the robot body center, on the ground.
        if robot:
            self.odom_T_task = get_root_T_ground_body(robot_state=self.robot.robot_state_client.get_robot_state(),
                                                root_frame_name=GRAV_ALIGNED_BODY_FRAME_NAME)

        # Set our tool frame to be the tip of the robot's bottom jaw. Flip the orientation so that
        # when the hand is pointed downwards, the tool's z-axis is pointed upward.
        self.wr1_T_tool = SE3Pose(0, 0, 0, Quat.from_pitch(-math.pi / 2))
        
        self.frequency_pose_count = int(60 // DIRECT_CONTROL_FREQUENCY)
        self.pose_receive_count = 0
        
        
        ## HTC tracker pose init
        self.pose = [0,0,0,0,0,0]
            
    def callback_action(self, data):
        print(f"\nI heard: {data.data}")
        if data.data == "stop_action" and self.robot.current_state_direct_control:
            self.robot.stop_direct_control()
            self.robot.current_state_direct_control = False
            try_state_send(self.sm, data.data)
        elif self.robot.current_state_direct_control:
            pass
        else:
            try_state_send(self.sm, data.data)
        
    def callback_gripper(self, data):
        if self.robot.current_state_direct_control:
            close_or_open = data.data
            self.robot.gripper(close_or_open)
        else:
            pass
        
    
    def callback_hand_pose(self, data):
        if self.robot.current_state_direct_control:
            if self.robot.init_pos_empty:
                self.arm_pos_init = [data.position.x, data.position.y, data.position.z]
        
            pos = 1.0*(np.array([data.position.x, data.position.y, data.position.z] - np.array(self.arm_pos_init)))
            orientation = math_helpers.Quat(1, 0, 0, 0)
            hand_pose = math_helpers.SE3Pose(x=0.75+pos[0], y=pos[1], z=0.45+pos[2], rot=orientation)
            
            self.pose_receive_count += 1
            if self.pose_receive_count >= self.frequency_pose_count:
                print(hand_pose)
                self.pose_receive_count = 0
                self.robot.init_pos_empty = False
                self.robot.move_to_cartesian_pose_rt_task(hand_pose, self.odom_T_task, self.wr1_T_tool)
        
        # end func
        
    def callback_hand_pose_trajectory(self, data):
        if self.robot.current_state_direct_control:
            if self.robot.init_pos_empty:
                self.arm_pos_init = [data.position.x, data.position.y, data.position.z]
                self.arm_ori_init = math_helpers.Quat(
                    w = data.orientation.w,
                    x = data.orientation.x,
                    y = data.orientation.y,
                    z = data.orientation.z
                )
                self.robot.init_pos_empty = False
        
            pos = 10*(np.array([data.position.x, data.position.y, data.position.z] - np.array(self.arm_pos_init)))
            
            quaternion = math_helpers.Quat(
                w = data.orientation.w,
                x = data.orientation.x,
                y = data.orientation.y,
                z = data.orientation.z
            )
            orientation = math_helpers.Quat()
            
            self.arm_pos_init = [data.position.x, data.position.y, data.position.z]
            
            hand_pose = math_helpers.SE3Pose(x=pos[0], y=pos[1], z=pos[2], rot=orientation)
            traj_point = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose.to_proto())
            self.direct_control_trajectory_list.append(traj_point)
            
            if len(self.direct_control_trajectory_list) >= 12:
                zero_pose = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
                zero_trajectory_pose = trajectory_pb2.SE3TrajectoryPoint(pose=zero_pose.to_proto())
                print(f"Executing path, total waypoint count {len(self.direct_control_trajectory_list)}")
                hand_traj = trajectory_pb2.SE3Trajectory(points=self.direct_control_trajectory_list)
                
                zero_pose = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
                zero_trajectory_pose = trajectory_pb2.SE3TrajectoryPoint(pose=zero_pose.to_proto())
                self.direct_control_trajectory_list = [zero_trajectory_pose]
                
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(pose_trajectory_in_task=hand_traj, root_frame_name="hand") #  "flat_body")
                
                # Pack everything up in protos.
                arm_command = arm_command_pb2.ArmCommand.Request(arm_cartesian_command=arm_cartesian_command)
                synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
                robot_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
                # print('Sending trajectory command... \n')

                # Send the trajectory to the robot.
                self.robot.command_client.robot_command(robot_command)
            
        else:
            pass
        
    def triangulate_position(self, data):
        pose = self.pose
        
        x_person = data.data[0]
        y_person = data.data[1]
        
        x_obj = data.data[3]
        y_obj = data.data[4]
        
        yaw_robot = pose[3]
        
        vector_person_robot = np.array([pose[0] - x_person, pose[1] - y_person])
        vector_person_object = np.array([x_obj - x_person, y_obj - y_person])
        vector_robot_object = vector_person_object - vector_person_robot
        
        rotation = np.arctan(vector_robot_object[0]/vector_robot_object[1]) - yaw_robot
        
        return vector_robot_object[0], vector_robot_object[1], rotation
        
    def callback_tracker_pose(self, data):
        print(f"Tracker pose: {data.data}")
        self.pose = data.data
        
    def callback_deictic_pickup(self, data):
        x, y, yaw = self.triangulate_position(data)
        print("Walking to x:{x}, y:{y}, angle:{yaw}")
        self.robot.two_d_location_body_frame_command(x, y, yaw)
        
        self.pick_up_object_in_front_of_robot()
        
        return
    
    def callback_deictic_dropoff(self, data):
        x, y, yaw = self.triangulate_position(data)
        print("Walking to x:{x}, y:{y}, angle:{yaw}")
        self.robot.two_d_location_body_frame_command(x, y, yaw)
        
        self.drop_off_object_in_front_of_robot()
        
        return
    
    def callback_deictic_walk(self, data):
        x, y, yaw = self.triangulate_position(data)
        print("Walking to x:{x}, y:{y}, angle:{yaw}")
        self.robot.two_d_location_body_frame_command(x, y, yaw)
        
        return
    
    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback_action)
        rospy.Subscriber("gripper", String, self.callback_gripper)3
        rospy.Subscriber("hand_pose", Pose, self.callback_hand_pose)
        rospy.Subscriber("robot_pose", Float32MultiArray, self.callback_tracker_pose)
        # rospy.Subscriber("deictic_pick_up", Pose, self.callback_deictic_pickup)
        # rospy.Subscriber("deictic_drop_off", Pose, self.callback_deictic_dropoff)
        rospy.Subscriber("deictic_walk", Pose, self.callback_deictic_walk)
        rospy.spin()


if __name__ == "__main__":

    # robotInterface = SpotControlInterface(DIRECT_CONTROL_FREQUENCY)
    robotInterface = None
    
    if robotInterface:
        sdk = bosdyn.client.create_standard_sdk('SpotControlInterface')
        robot = sdk.create_robot("192.168.61.157")
        bosdyn.client.util.authenticate(robot)
        robot.time_sync.wait_for_sync()
        assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."
        
        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        lease_client.take()
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            robot.logger.info("Powering on robot... This may take several seconds.")
            robot.power_on(timeout_sec=20)
            assert robot.is_powered_on(), "Robot power on failed."
            robot.logger.info("Robot powered on.")

            robotInterface.image_client = robot.ensure_client(ImageClient.default_service_name)

            # Create a command client to be able to command the robot
            robotInterface.command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            robotInterface.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
            robotInterface.manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)
            
            robotInterface.stand(0.0)
            time.sleep(1)

            fsm = FsmNode(robot=robotInterface) 
            fsm.run()
    else:
        fsm = FsmNode(robot=robotInterface) 
        fsm.run()
        
    # robotInterface.sit_down()