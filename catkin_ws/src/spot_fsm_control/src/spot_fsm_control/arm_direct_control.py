import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class SpotDirectArmControl:
    
    def __init__(self, robot):
        self.robot = robot
        self.current_state = "stand"
    
    def callback_gripper(self, data):
        close_or_open = data.data
        self.robot.gripper(close_or_open)
    
    def callback_hand_pose(self, data):
        position = [data.position.x, data.position.y, data.position.z]
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.robot.direct_control(position, quaternion)
        
    def run(self):
        self.robot.ready_or_stow_arm()
        rospy.init_node('direct_arm_control', anonymous=True)
        while not rospy.is_shutdown():
            rospy.Subscriber("gripper", String, self.callback_gripper)
            rospy.Subscriber("hand_pose", Pose, self.callback_hand_pose)
            if self.current_state != "direct_arm_control":
                rospy.signal_shutdown()
                
        self.robot.ready_or_stow_arm(stow=True)
