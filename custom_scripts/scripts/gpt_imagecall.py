#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import subprocess
import json
import inspect

LOG_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/log.json"
PYTHON_VERSION = "python3.7"
PYTHON3_7_SCRIPT_PATH = "/home/student/ros_ws/src/custom_scripts/python3_scripts/gpt_imagecall_py3.py"
STARTUP_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/startup.json"
transcription_buffer = None
img_buffer = None
publisher = None

class GPTError(Exception):
    def __init__(self, message="There was an error during the GPT process."):
        self.message = message
        super(GPTError, self).__init__(self.message)

def call_gpt_py3():
    """
    Given both transcription text and image(s) have been received, 
    start GPT API call.
    Because ROS does not support Python 3 and OpenAI package, call subprocess
    to find transcription.txt and transcription_img.jpg and use those.
    Uses startup.json place before user input to provide context and few-shot learning.
    Append GPT response to conversation log.
    """
    global publisher
    file_name = inspect.getfile(inspect.currentframe())
    error_pub = rospy.Publisher('/gpt_errors', String, queue_size=10)
    
    try:
        output = subprocess.check_output([PYTHON_VERSION, PYTHON3_7_SCRIPT_PATH])
        rospy.loginfo("gpt_imagecall.py received API response:\n{}".format(output))
        
        messages = []
        with open(LOG_PATH, 'r') as log:
            messages = json.load(log)
        messages.append(
            {
                "role": "assistant",
                "content": {"type": "text", "text": output},
            }
        )
        with open(LOG_PATH, 'w') as log:
            json.dump(messages, log)
        publisher.publish(output)

    except subprocess.CalledProcessError as e:
        error_message = "GPTError in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        error_pub.publish(error_message)
    except Exception as e:
        error_message = "An unexpected error occurred in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        error_pub.publish(error_message)

def check_img_received(text):
    """
    After receiving transcription text, check to see if image has been received.
    If yes, start GPT API call, then empty buffers.
    If no, load buffer and wait.
    """
    global transcription_buffer, img_buffer
    transcription_buffer = text.data
    if img_buffer is None:
        rospy.loginfo("gpt_imagecall.py received transcription, waiting for image")
    else:
        call_gpt_py3()
        transcription_buffer = None
        img_buffer = None

def check_transcription_received(img):
    """
    After receiving transcription image, check to see if text has been received.
    If yes, start GPT API call, then empty buffers.
    If no, load buffer and wait.
    """
    global transcription_buffer, img_buffer
    img_buffer = img.data
    if transcription_buffer is None:
        rospy.loginfo("gpt_imagecall.py received image, waiting for transcription")
    else:
        call_gpt_py3()
        transcription_buffer = None
        img_buffer = None

def main():
    rospy.init_node('gpt_imagecall')

    rospy.loginfo("gpt_imagecall.py entered, looking for publish on /transcription and /transcription_img")
    try:
        with open(STARTUP_PATH, 'rb') as startup_file:
            startup = json.load(startup_file)
        with open(LOG_PATH, 'w') as log:
            json.dump(startup, log)

    except Exception as e:
        rospy.logerr("An error occurred while initializing: {}".format(e))

    global publisher
    publisher = rospy.Publisher("/gpt_response", String, queue_size=10)

    rospy.Subscriber("/transcription", String, check_img_received, queue_size=2)
    rospy.Subscriber("/transcription_imgs", Image, check_transcription_received)
    rospy.spin()

    rospy.loginfo("gpt_imagecall.py exiting")

if __name__ == '__main__':
    main()