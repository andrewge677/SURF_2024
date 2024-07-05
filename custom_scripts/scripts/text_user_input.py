#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def input_talker():
    # Initialize the ROS node named 'speech_to_text'
    rospy.init_node('speech_to_text', anonymous=True)
    # Create a publisher that will publish on the 'transcription' topic
    pub = rospy.Publisher('transcription', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        input_str = raw_input("Requesting Prompt:")

        if(input_str == "quit" or input_str == "exit"):
            break

        pub.publish(input_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        input_talker()
    except rospy.ROSInterruptException:
        pass
