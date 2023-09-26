#!/usr/bin/env python

import time
import math
import rospy
from std_msgs.msg import Float32MultiArray
from  htc_vive_tracker import triad_openvr



class HTCViveTrackerNode:

    def __init__(self):
        ## HTC tracker init
        self.vive_tracker = triad_openvr.triad_openvr()
        self.vive_tracker.print_discovered_objects()
        
        self.pub = rospy.Publisher('robot_pose', Float32MultiArray, queue_size=100)
        rospy.init_node('robot_pose_node', anonymous=True)
            
    def run(self):
        rate = rospy.Rate(1)
        while True:
            pose = self.vive_tracker.devices["tracker_1"].get_pose_euler()
            msg = Float32MultiArray()
            msg.data = pose
            print(f"\rPose: {pose}", end="")
            self.pub.publish(msg)
            rate.sleep()

        

if __name__ == "__main__":
    node =  HTCViveTrackerNode()
    node.run()