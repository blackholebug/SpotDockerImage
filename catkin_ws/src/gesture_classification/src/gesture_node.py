#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from joblib import load

from gesture_classification.gesture_classification import GestureClassification


class GestureClassificationNode:
    """_summary_
    FPS = 60 frames / 15 remove six keypoint collections every classification round = 4 classification inferences as second
    
    """
    
    def __init__(self):
        self.hand_keypoint_list = []
        self.model = GestureClassification()
        self.img_idx = 1
        self.clf = load("/catkin_ws/src/gesture_classification/models/KNN_120.joblib")
        
    def classify_time_series(self, series):
        predictions = []
        for hand in series:
            hand = np.array(hand).reshape(1, -1)
            y = self.clf.predict(hand)
            predictions.append(y[0])
        
        sign = max(set(predictions), key=predictions.count)
        print(predictions)
        
        return sign
    
    def callback(self, data):
        array = np.array(data.data)
        self.hand_keypoint_list.append(array)
        
        if len(self.hand_keypoint_list) > 60:
            series = self.hand_keypoint_list.copy()
            self.hand_keypoint_list = self.hand_keypoint_list[30:]
            gesture = self.classify_time_series(series)
            print(gesture) ## publisher this gesture

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("hand_keypoints", Float32MultiArray, self.callback)
        rospy.spin()


if __name__ == "__main__":

    gc = GestureClassificationNode()
    gc.run()