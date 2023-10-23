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
        self.incoming_gestures = []
        self.pub = rospy.Publisher('chatter', String, queue_size=60)
        self.serie_size = 60
        
        self.recognition_frequency = 2 # Hz
        self.slice_size = int( self.serie_size // self.recognition_frequency )
    
    def callback(self, data):
        self.incoming_gestures.append(data.data)
        
        if len(self.incoming_gestures) >= self.serie_size:
            list_to_classify = self.incoming_gestures.copy()
            self.incoming_gestures = self.incoming_gesture[self.slice_size:]
            action = max(set(list_to_classify), key=list_to_classify.count)
            count_action = list_to_classify.count(action)
            if count_action > int(self.serie_size*0.66667):
                if action == "no gesture":
                    pass
                else:
                    self.pub.publish(action)

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("gesture_recognition", String, self.callback)
        rospy.spin()
            


if __name__ == "__main__":
    gc = GestureClassificationNode()
    gc.run()