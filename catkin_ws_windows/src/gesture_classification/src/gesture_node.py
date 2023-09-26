#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from joblib import load
from sklearn import preprocessing as pre
from scipy.spatial.transform import Rotation as R
import json
from datetime import datetime

class GestureClassificationNode:
    """_summary_
    FPS = 60 frames / 15 remove six keypoint collections every classification round = 4 classification inferences as second
    
    """
    
    def __init__(self):
        self.hand_keypoint_list = []
        self.img_idx = 1
        self.clf = load("/catkin_ws/src/gesture_classification/models/SVM.joblib")
        
        self.incoming_gestures = []
        self.pub = rospy.Publisher('chatter', String, queue_size=60)
        self.serie_size = 60
        
        self.recognition_frequency = 2 # Hz
        self.slice_size = int( self.serie_size // self.recognition_frequency )
        
        self.keypoint_dictionary = {}
        self.i = 0
        self.signs = [
            "call",
            "dislike",
            "fist",
            "four", 
            "like", 
            "mute",
            "ok", 
            "one",
            "palm", 
            "peace",
            "peace inverted", 
            "rock", 
            "stop",
            "stop inverted",
            "three",
            "three 2",
            "two up",
            "two up inverted",
        ]
        self.total_signs_to_record = len(self.signs) * 3
        
    def classify_time_series(self, series):
        hand = np.array(series).reshape(len(series), -1)
        y = self.clf.predict(hand)
        predictions = y.tolist()
        sign = max(set(predictions), key=predictions.count)
        print(predictions)
        
        return sign
    
    def callback(self, data):
        self.incoming_gestures.append(data.data)
        
        if len(self.incoming_gestures) >= self.serie_size:
            list_to_classify = self.incoming_gestures.copy()
            self.incoming_gestures = self.incoming_gesture[30:]
            action = max(set(list_to_classify), key=list_to_classify.count)
            count_action = list_to_classify.count(action)
            if count_action > int(self.serie_size*0.66667):
                if action == "no gesture":
                    pass
                else:
                    self.pub.publish(action)
        
        
    
    def callback_keypoints(self, data):
        array = np.array(data.data)
        self.hand_keypoint_list.append(array)
        
        if len(self.hand_keypoint_list) > self.serie_size:
            series = self.hand_keypoint_list.copy()
            self.hand_keypoint_list = self.hand_keypoint_list[self.slice_size:]
            gesture = self.classify_time_series(series)
            print(gesture) # publisher this gesture
            
            
    def callback_save_pose(self, data):
        array = np.array(data.data).reshape(21, 3)
        keypoints = array.tolist()
        
        
        if self.i >= self.total_signs_to_record:
            sign = "no gesture"
        else:
            sign = self.signs[int(self.i // 3)]
        
        if (int(self.i % 3) == 0) and (sign != "no gesture"):
            self.keypoint_dictionary[sign] = {}  
        elif self.i == self.total_signs_to_record:
            self.keypoint_dictionary[sign] = {}

        print(f"{sign} #{int(self.i % 3)+1}")
        self.keypoint_dictionary[sign][self.i] = {}
        self.keypoint_dictionary[sign][self.i]["keypoints"] = keypoints
        self.keypoint_dictionary[sign][self.i]["labels"] = [sign]
        self.keypoint_dictionary[sign][self.i]["leading_hand"] = "right"
        self.keypoint_dictionary[sign][self.i]["hand_order"] = ["Right"]
        self.keypoint_dictionary[sign][self.i]["number_of_hands_detected"] = 1


        self.i += 1
        
        date_time = datetime.now().strftime("%m-%d-%Y-%H:%M:%S")
        with open(f"/catkin_ws/P05-{date_time}.json", "w") as outfile:
            json.dump(self.keypoint_dictionary, outfile)
    

    def run(self):
        date_time = datetime.now().strftime("%m-%d-%Y-%H:%M:%S")
        with open(f"/catkin_ws/P05-{date_time}.json", "w") as outfile:
            json.dump(self.keypoint_dictionary, outfile)
        rospy.init_node('listener', anonymous=True)
        
        rospy.Subscriber("gesture_recognition", String, self.callback)
        # rospy.Subscriber("hand_keypoints", Float32MultiArray, self.callback_keypoints)
        # rospy.Subscriber("hand_keypoints", Float32MultiArray, self.callback_save_pose)
        rospy.spin()
        
        date_time = datetime.now().strftime("%m-%d-%Y-%H:%M:%S")
        with open(f"/catkin_ws/P05-{date_time}.json", "w") as outfile:
            json.dump(self.keypoint_dictionary, outfile)
            


if __name__ == "__main__":

    gc = GestureClassificationNode()
    gc.run()