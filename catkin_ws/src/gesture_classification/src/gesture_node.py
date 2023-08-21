#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

from gesture_classification.gesture_classification import GestureClassification


class GestureClassificationNode:
    """_summary_
    FPS = 60 frames / 15 remove six keypoint collections every classification round = 4 classification inferences as second
    
    """
    
    def __init__(self):
        self.hand_keypoint_list = []
        self.model = GestureClassification()
        self.img_idx = 1
        pass
    
    def callback(self, data):
        array = np.array(data.data).reshape(21, 3)
        np.save
        self.hand_keypoint_list.append(array)
        
        if len(self.hand_keypoint_list) > 60:
            series = self.hand_keypoint_list.copy()
            self.hand_keypoint_list = self.hand_keypoint_list[15:]
            gesture = self.model.classify_time_series(series)
            print(gesture)

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("hand_keypoints", Float32MultiArray, self.callback)
        rospy.spin()


if __name__ == "__main__":

    gc = GestureClassificationNode()
    gc.run()