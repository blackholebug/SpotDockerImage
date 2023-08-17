#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray

class GestureClassification:
    
    def __init__(self):
        pass
    
    def callback(self, data):
        print("Received:", data.data)
        time.sleep(1)

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("hand_keypoints", Float32MultiArray, self.callback)


if __name__ == "__main__":

    gc = GestureClassification()
    gc.run()