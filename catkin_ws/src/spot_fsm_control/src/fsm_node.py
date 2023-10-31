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
import csv
import numpy as np
import logging
from datetime import datetime
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
from bosdyn.client.math_helpers import quat_to_eulerZYX
from bosdyn.client.math_helpers import Quat, SE3Pose

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b, get_odom_tform_body
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

    def __init__(self, robot: SpotControlInterface, robot_sdk):
        self.robot = robot
        self.robot_sdk = robot_sdk
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
        
        ## Odometry pose init
        self.pose = np.array([0, 0, 0, 0, 0, 0])
        
        self.initial_position_vision_odom = np.array([0, 0, 0, 0, 0, 0])
        self.initial_position_odom = np.array([0, 0, 0, 0, 0, 0])
        self.start_position_offset = np.array([1.2, 0, 0, 0, 0, 0])
        
        self.correction_yaw_odom = 0
        self.correction_yaw_vision = 0
        self.current_yaw_state = 0 # in radians
            
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
            
    def callback_action_dummy(self, data):
        print(f"\nI heard: {data.data}")
        
    def triangulate_position(self, data):
        pose = self.get_robot_vision_pose()
        # pose = self.get_robot_odom_pose()
        
        x_person = data.data[0]
        y_person = data.data[1]
        
        x_obj = data.data[3]
        y_obj = data.data[4]
        
        vector_person_robot = np.array([pose[0] - x_person, pose[1] - y_person])
        vector_person_object = np.array([x_obj - x_person, y_obj - y_person])
        
        # vector_robot_object = vector_person_object - vector_person_robot ## vector calculation  
        vector_robot_object = np.array([x_obj - pose[0], y_obj - pose[1]]) # determining relative movement directly fron goal position to starting position
        
        print("Vector person --> Robot:", vector_person_robot)
        print("Vector person --> Object:", vector_person_object)
        
        x, y = self.rotation_matrix_calc(vector_robot_object[0], vector_robot_object[1], -1*pose[3])
        
        if vector_robot_object[0] >= 0:
            rotation = np.arctan(y/x) 
        if vector_robot_object[0] < 0 and vector_robot_object[1] >= 0:
            rotation = np.arctan(y/x) + np.pi    
        elif vector_robot_object[0] < 0 and vector_robot_object[1] < 0:
            rotation = np.arctan(y/x) - np.pi      
        
        data_to_save = [x_person, y_person, x_obj, y_obj, rotation, pose[0], pose[1]]
        
        return x, y, rotation, data_to_save
    
    def get_robot_vision_pose(self):
        vision_T_body = get_a_tform_b(self.robot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot,"vision","body") 
        visionBodyEuler = quat_to_eulerZYX(vision_T_body.rot)
        pose_raw = [vision_T_body.x, vision_T_body.y, vision_T_body.z, visionBodyEuler[0], visionBodyEuler[1], visionBodyEuler[2]]
        
        pose = self.pose_transformation_vision(pose_raw)
        return pose

    def pose_transformation_vision(self, data):
        data_zero = np.array(data) - self.initial_position_vision_odom
        x, y = self.rotation_matrix_calc(data_zero[0], data_zero[1], self.correction_yaw_vision)
        z = data_zero[2]
        
        data_transformed = [x, y, z, data_zero[3], data_zero[4], data_zero[5]]

        return np.array(data_transformed) + self.start_position_offset
    
    def get_robot_odom_pose(self):
        odom_T_body = get_a_tform_b(self.robot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot,"odom","body") 
        visionBodyEuler = quat_to_eulerZYX(odom_T_body.rot)
        pose_raw = [odom_T_body.x, odom_T_body.y, odom_T_body.z, visionBodyEuler[0], visionBodyEuler[1], visionBodyEuler[2]]
        
        pose = self.pose_transformation_odom(pose_raw)
        return pose

    def pose_transformation_odom(self, data):
        data_zero = np.array(data) - self.initial_position_odom
        x, y = self.rotation_matrix_calc(data_zero[0], data_zero[1], self.correction_yaw_odom)
        z = data_zero[2]
        
        data_transformed = [x, y, z, data_zero[3], data_zero[4], data_zero[5]]

        return np.array(data_transformed) + self.start_position_offset
        

    def rotation_matrix_calc(self, x, y, rotation):
        x = np.cos(rotation) * x - np.sin(rotation) * y
        y = np.sin(rotation) * x + np.cos(rotation) * y
        return x, y
        
    def calibrate_odometry_rotations(self, calibration_poses, frame="odom"):
        try:
            yaw_per_pose = [] # yaw in degrees
            for pose in calibration_poses[5:]:
                x = pose[0]
                y = pose[1]
                if x >= 0:
                    yaw_radian = np.arctan(y/x)
                if x < 0 and y >= 0:
                    yaw_radian = np.arctan(y/x) + np.pi
                elif x < 0 and y < 0:
                    yaw_radian = np.arctan(y/x) - np.pi
                    
                yaw_per_pose.append(yaw_radian)
                
            if frame == "odom":
                self.correction_yaw_odom = np.average(yaw_per_pose) * -1
                print("Correction YAW ODOM degrees: ", np.rad2deg(self.correction_yaw_odom))
            elif frame == "vision":
                self.correction_yaw_vision = np.average(yaw_per_pose) * -1
                print("Correction YAW VISION degrees: ", np.rad2deg(self.correction_yaw_vision)) 
            
        except Exception as e:
            print(e)
        
        return
        
    def calibration_movement(self):
        odom_positions, vision_odom_positions = self.robot.calibration_movement_in_robot_frame()
        
        self.initial_position_odom = np.array(odom_positions[0])
        self.initial_position_vision_odom = np.array(vision_odom_positions[0])
        
        return odom_positions, vision_odom_positions
    
    def callback_deictic_walk(self, data):
        x, y, yaw, data_to_save = self.triangulate_position(data)
        print(f"Walking to x:{x}, y:{y}, angle:{yaw}")
        
        self.robot.two_d_location_body_frame_command(x, y, yaw)
        time.sleep(1)
         
        new_pose = self.get_robot_vision_pose()
        print("New robot pose: ", new_pose)
        
        data_to_save.append(new_pose[0])
        data_to_save.append(new_pose[1])
        data_to_save.append(new_pose[3])
        with open(self.csv_filename , 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data_to_save)
            file.close()
        
    def run(self):
        date_start = datetime.now().strftime("%Y-%m-%dT%H%M%S")
        self.csv_filename = f"C:\\dev\\SpotDockerImage\\data\deictic_movements\\deictic_data_{date_start}.csv"
        with open(self.csv_filename , 'w', newline='') as file:
            writer = csv.writer(file)
            field = ["x_person", "y_person", "x_object", "y_object", "goal_rotation", "x_robot", "y_robot", "x_robot_new", "y_robot_new", "robot_rotation_new"]
            writer.writerow(field)
            file.close()

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback_action)
        rospy.Subscriber("deictic_walk", Float32MultiArray, self.callback_deictic_walk)
        
        time.sleep(1)
        odom_positions, vision_odom_positions = self.calibration_movement()
        self.calibrate_odometry_rotations(odom_positions, frame="odom")
        self.calibrate_odometry_rotations(vision_odom_positions, frame="vision")
        
        self.robot.two_d_location_body_frame_command(x=-1.0, y=0, yaw=0)
        self.robot.sit_down()
        
        print("Calibration Finished.")
        
        rospy.spin()
    
    def run_dummy(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback_action_dummy)
        rospy.spin()

        

if __name__ == "__main__":

    robotInterface = SpotControlInterface(DIRECT_CONTROL_FREQUENCY)
    # robotInterface = None
    
    if robotInterface:
        sdk = bosdyn.client.create_standard_sdk('SpotControlInterface')
        # robot = sdk.create_robot("192.168.31.214")
        robot = sdk.create_robot("192.168.20.157")
        robotInterface.robot_sdk = robot
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
            
            time.sleep(1)

            fsm = FsmNode(robot=robotInterface, robot_sdk=robot)
            fsm.run()
    else:
        fsm = FsmNode(robot=robotInterface, robot_sdk=None) 
        fsm.run_dummy()
        
    # robotInterface.sit_down()