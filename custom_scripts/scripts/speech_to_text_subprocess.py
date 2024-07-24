#!/usr/bin/env python

# currently under construction because speech_to_text_base is the only program that accurately cuts audio files when pause is detected.

import rospy
from std_msgs.msg import String
import subprocess

def recognize_speech():
    try:
        # this line runs a python3 file because ROS requires python2
        output = subprocess.check_output(['python3', '/home/student/ros_ws/src/custom_scripts/python3_scripts/speech_rec_py3.py'])
        return output.strip()
    except subprocess.CalledProcessError as e:
        rospy.logerr("Error calling speech recognition script: {}".format(e.output))
        return ''

def speech_recognition_node():
    rospy.init_node('speech_recognition_node')
    pub = rospy.Publisher('/transcription', String, queue_size=10)
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

