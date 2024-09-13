#!/usr/bin/env python

# currently under construction because speech_to_text_base is the only program that accurately cuts audio files when pause is detected.

import rospy
from std_msgs.msg import String
import requests
import os
import inspect

TRANSCRIPTION_PATH = '/home/student/ros_ws/src/custom_scripts/transcripts/transcription.txt'
robot_is_free = True

class SpeechRecognitionError(Exception):
    def __init__(self, message="An error occurred during speech recognition."):
        self.message = message
        super(SpeechRecognitionError, self).__init__(self.message)

def recognize_speech():
    try:
        response = requests.post('http://localhost:8080')
        data = response.json()
        return data.get('transcript', '')
    except requests.exceptions.RequestException as e:
        file_name = inspect.getfile(inspect.currentframe())
        error_message = "Error calling speech recognition server in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        raise SpeechRecognitionError(error_message)
    
def robot_is_free(object):
    print("Robot is listening again.")
    global robot_is_free
    robot_is_free = True

def speech_client_node():
    global robot_is_free
    rospy.init_node('speech_client_node')
    transcription_publisher = rospy.Publisher('/transcription', String, queue_size=1)
    rospy.Subscriber("/robot_finished_step", String, robot_is_free)
    rate = rospy.Rate(0.1)  # Adjust the rate as needed (0.1 Hz means it will run every 10 seconds)

    while not rospy.is_shutdown():
        try:
            recognized_text = recognize_speech()
            if recognized_text:
                rospy.loginfo("ROS loginfo: {}".format(recognized_text))
                if not robot_is_free:
                    rospy.loginfo("ROS loginfo: {}".format("transcription received but robot was busy"))
                else:
                    with open(TRANSCRIPTION_PATH, 'w') as f:
                        f.write(recognized_text)
                        os.chmod(TRANSCRIPTION_PATH, 0o600)  # Owner read/write (rw-------)

                    transcription_publisher.publish(recognized_text)
                    robot_is_free = False
        except SpeechRecognitionError as e:
            rospy.logerr(e.message)
        except Exception as e:
            file_name = inspect.getfile(inspect.currentframe())
            error_message = "An unexpected error occurred in {}: {}".format(file_name, e)
            rospy.logerr(error_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Speech to text node initializing")
        speech_client_node()
    except rospy.ROSInterruptException as e:
        file_name = inspect.getfile(inspect.currentframe())
        rospy.logerr("ROSInterruptException in {}: {}".format(file_name, e))
