#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def error_callback(msg):
    rospy.logerr("Error detected: {}".format(msg.data))

def error_listener():
    rospy.init_node('error_listener', anonymous=True)
    
    # Subscribe to all error topics
    rospy.Subscriber('/speak_errors', String, error_callback)
    rospy.Subscriber('/instructions_errors', String, error_callback)
    rospy.Subscriber('/gpt_errors', String, error_callback)
    rospy.Subscriber('/photo_errors', String, error_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        error_listener()
    except rospy.ROSInterruptException:
        pass
