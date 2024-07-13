#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import os

TRANSCRIPTION_PATH = '/home/student/ros_ws/src/custom_scripts/transcripts/transcription.txt'

def recognize_speech():
    try:
        response = requests.post('http://localhost:8080')
        data = response.json()
        return data.get('transcript', '')
    except requests.exceptions.RequestException as e:
        rospy.logerr("Error calling speech recognition server: {}".format(e))
        return ''

def speech_client_node():
    rospy.init_node('speech_client_node')
    pub = rospy.Publisher('transcription', String, queue_size=10)
    rate = rospy.Rate(0.1)  # Adjust the rate as needed (0.1 Hz means it will run every 10 seconds)

    while not rospy.is_shutdown():
        recognized_text = recognize_speech()
        print("Received: {}", recognized_text)
        if recognized_text:
            rospy.loginfo("ROS loginfo: {}".format(recognized_text))
	
        with open(TRANSCRIPTION_PATH, 'w') as f:
            f.write(recognized_text)
            os.chmod(TRANSCRIPTION_PATH, 0o600)  # Owner read/write (rw-------)

        pub.publish(recognized_text)
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Speech to text node initializing")
        speech_client_node()
    except rospy.ROSInterruptException:
        pass
