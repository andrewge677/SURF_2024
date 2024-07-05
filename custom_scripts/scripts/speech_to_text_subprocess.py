#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

def recognize_speech():
    try:
        # this line reads a python3 file to prevent ROS conflicts
        output = subprocess.check_output(['python3', '/home/student/ros_ws_py3/src/gspeech-master/scripts/speech_rec.py'])
        return output.strip()
    except subprocess.CalledProcessError as e:
        rospy.logerr("Error calling speech recognition script: {}".format(e.output))
        return ''

def speech_recognition_node():
    rospy.init_node('speech_recognition_node')
    pub = rospy.Publisher('transcription', String, queue_size=10)
    rate = rospy.Rate(0.1)  # Adjust the rate as needed (0.1 Hz means it will run every 10 seconds)

    while not rospy.is_shutdown():
        recognized_text = recognize_speech()
        if recognized_text:
            rospy.loginfo("Publishing recognized text: {}".format(recognized_text))
            pub.publish(recognized_text)
        rate.sleep()

if __name__ == '__main__':
    try:
        speech_recognition_node()
    except rospy.ROSInterruptException:
        pass

