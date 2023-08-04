#!/usr/bin/env python

import time
import sys
sys.path.append("./hagrid/")
sys.path.append("./src/")

import rospy
from std_msgs.msg import String
import ast

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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from spot_fsm_control.spot_control_interface import SpotControlInterface
from spot_fsm_control.finite_state_machine import SpotStateMachine

logging.basicConfig(format="[LINE:%(lineno)d] %(levelname)-8s [%(asctime)s]  %(message)s", level=logging.INFO)

def try_state_send(state_machine, action):
    print("Current state:", state_machine.current_state)
    try:
        print(f"trying to: .{action}.")
        state_machine.send(action)
    except:
        try:
            state_machine.send("stop_action")
            print("Stop action first")
            state_machine.send(action)
        except:
            try:
                state_machine.send("stand_up")
                print("Stand up first")
                state_machine.send(action)
            except:
                print(f"{action} not possible")
                ## Do some handling or more feedback to user


class FsmNode:

    def __init__(self, robot: SpotControlInterface):
        self.sm = SpotStateMachine(robot=robot)
            
    def callback(self, data):
        print(f"\nI heard: {data.data}")
        try_state_send(self.sm, data.data)
    
    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback)
        rospy.spin()


if __name__ == "__main__":

    # robotInterface = SpotControlInterface()
    robotInterface = None
    
    if robotInterface:
        sdk = bosdyn.client.create_standard_sdk('SpotControlInterface')
        robot = sdk.create_robot("192.168.196.157")
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