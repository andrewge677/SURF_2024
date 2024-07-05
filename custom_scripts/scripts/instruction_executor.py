#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import custom_scripts.scripts.close_gripper as close_gripper
import custom_scripts.scripts.open_gripper as open_gripper
import take_photo
import subprocess

def callback(msg):

    if(msg.data == "close_gripper"):
        subprocess.call(['rosrun', 'custom_scripts', 'close_gripper_2.py'])
    elif(msg.data == "open_gripper"):
        subprocess.call(['rosrun', 'custom_scripts', 'open_gripper_2.py'])
    elif(msg.data == "take_photo"):
        subprocess.call(['rosrun', 'custom_scripts', 'take_photo.py', 'inst_test'])
    else:
        rospy.loginfo("Command not recognized.")

def main():
    rospy.init_node('instruction_executor', anonymous=True)
    rospy.Subscriber('/chatgpt_response', String, callback)
    pub = rospy.Publisher('/instructions_errors', String, queue_size=10)

    rospy.loginfo("Robot now listening to /chatgpt_response for instructions...")
    rospy.spin()

if __name__ == '__main__':
    main()