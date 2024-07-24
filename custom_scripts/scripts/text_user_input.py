#!/usr/bin/env python
import rospy
from std_msgs.msg import String

TRANSCRIPTION_PATH = '/home/student/ros_ws/src/custom_scripts/transcripts/transcription.txt'

def input_talker():
    rospy.init_node('speech_to_text', anonymous=True)
    rospy.loginfo("text_user_input.py entered. enter 'quit' or 'exit' to end.")
    pub = rospy.Publisher('transcription', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        input_str = raw_input("Requesting Prompt:")

        if(input_str == "quit" or input_str == "exit"):
            break

        with open(TRANSCRIPTION_PATH, 'w') as f:
            f.write(input_str) 
        pub.publish(input_str)
        rate.sleep()
    rospy.loginfo("text_user_input.py exiting")

if __name__ == '__main__':
    try:
        input_talker()
    except rospy.ROSInterruptException:
        pass
