#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import pickle
import sklearn
from scipy.spatial.transform import Rotation as R
from collections import Counter
from datetime import datetime

class GestureClassificationNode:
    """_summary_    
    """
    
    def __init__(self):
        self.clf = pickle.load(open("/catkin_ws/src/gesture_classification/models/LinearSVC.model", 'rb'))
        self.ppl = pickle.load(open("/catkin_ws/src/gesture_classification/models/training.ppl", 'rb'))
        self.pub = rospy.Publisher('/gesture_rec_out', String, queue_size=60)
        self.frame_buffer = []
        self.frame_sample_rate = 5
        self.is_debug = True

        # self.recognition_frequency = 2 # Hz
        # self.slice_size = int( self.serie_size // self.recognition_frequency )
        
        self.label_decoding=[
            'GoLeft',
            'GoRight',
            'TurnLeft',
            'TurnRight',
            'Forwards',
            'Backwards'
        ]

    def frequency_query(self, labels: np.ndarray):
        data = Counter(labels)
        return data.most_common(1)[0]


    def check_threshold(self, labels: np.ndarray, confidence: np.ndarray):
        # only when most labels are the same and the lowest confidence level is above 0.85, return the gesture
        invalid_gesture = "NoGesture"
        vote_threshold = 0.6
        conf_threshold = 0.93
        label, frequency = self.frequency_query(labels)
        if frequency/len(labels) < vote_threshold:
            return invalid_gesture
        if confidence.mean() < conf_threshold:
            return invalid_gesture
        print(f"current convidance: {confidence.mean()}")
        return self.label_decoding[label]
        

    def recognize_gesture(self):
        self.frame_buffer = np.array(self.frame_buffer).reshape(-1, 78)
        x = self.ppl.transform(self.frame_buffer)
        y = self.clf.predict_proba(x)
        labels = np.argmax(y, axis=1)
        confidence = np.max(y, axis=1)
        # if self.is_debug:
        #     for i in range(len(labels)):
        #         print(f"current label: {labels[i]}; current confidence: {confidence[i]} \n")
        res = self.check_threshold(labels, confidence)
        print(f"current gesture: {res}")
        self.pub.publish(f"{res}")

    def callback_gesture(self, data):
        if len(self.frame_buffer) < self.frame_sample_rate:
            self.frame_buffer.append(data.data)
        else:
            self.recognize_gesture()
            self.frame_buffer = []
    
    def callback_show_keypoints(self, data):
        array = np.array(data.data).reshape(-1,78)
        self.hand_keypoint_list.append(array)
        print(array)


    def run(self):
        date_time = datetime.now().strftime("%m-%d-%Y-%H:%M:%S")
        rospy.init_node('gesture_recognizer', anonymous=True)
        rospy.Subscriber("/hand_keypoints", Float32MultiArray, self.callback_gesture)
        rospy.spin()

if __name__ == "__main__":

    gc = GestureClassificationNode()
    gc.run()