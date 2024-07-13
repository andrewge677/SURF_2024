#!/usr/bin/env python

# import requests
# import json
# import cv2
# import base64
# import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import subprocess

# API_KEY = os.getenv("OPEN_AI_API_KEY")
# API_URL = 'https://api.openai.com/v1/chat/completions'
transcription_buffer = None
img_buffer = None
publisher = None
PYTHON_VERSION = "python3.7"
PYTHON3_7_SCRIPT_PATH = "/home/student/ros_ws_py3/src/gspeech-master/scripts/gpt_imagecall_py3.py"

# def gpt4_vision_response(image_path,prompt):
#     url = API_URL
#     headers = {
#         "Authorization": "Bearer {}".format(API_KEY),
#         "Content-Type": "application/json"
#     }
    
#     # Read image from file
#     image = cv2.imread(image_path)
#     max_dim = 512  # Maximum dimension for resizing 
#     height, width = image.shape[:2] 
#     if max(height, width) > max_dim: 
#         scale = max_dim / float(max(height, width)) 
#         new_size = (int(width * scale), int(height * scale)) 
#         image = cv2.resize(image, new_size)

#     _, img_encoded = cv2.imencode('.jpg', image)
#     img_base64 = base64.b64encode(img_encoded).decode('utf-8')

#     data = { "model": "gpt-4-turbo", 
#             "messages": [ {"role": "system", "content": "You are a helpful assistant."}, 
#             {"role": "user", "content": prompt}, 
#             {"role": "user", "content": "<img src='data:image/jpeg;base64,{}'>".format(img_base64)} ],"max_tokens": 150 }
    
#     response = requests.post(url, headers=headers, data=json.dumps(data))
#     response_json = response.json()
#     if 'choices' in response_json and len(response_json['choices']) > 0:
#         return response_json['choices'][0]['message']['content'].strip()
#     else:
#         print(response)
#         print(response_json)
#         return "Error processing image"

def call_gpt_py3():
    global publisher
    output = subprocess.check_output([PYTHON_VERSION, PYTHON3_7_SCRIPT_PATH])
    print(output)
    publisher.publish(output)

def check_img_received(text):
    global transcription_buffer, img_buffer
    transcription_buffer = text.data
    if(img_buffer == None):
        print("Waiting for img")
    else:
        # print(img_buffer, transcription_buffer)
        call_gpt_py3()
        transcription_buffer = None
        img_buffer = None

def check_transcription_received(img):
    global transcription_buffer, img_buffer
    img_buffer = img.data
    if(transcription_buffer == None):
        print("Waiting for transcription")
    else:
        # print(img_buffer, transcription_buffer)
        call_gpt_py3()
        transcription_buffer = None
        img_buffer = None

# TODO: needs checking for if two images/transcriptions sent before counterpart

if __name__ == '__main__':
    # # Path to the image file
    # image_path = '/home/student/Downloads/earth.jpg'
    # prompt = "Describe the contents of the image"
    
    # response_text = gpt4_vision_response(image_path,prompt)
    
    # if response_text:
    #     print("Response: {}".format(response_text))

    print("gpt_imagecall.py looking for transcription and transcription image.")
    rospy.init_node('gpt_imagecall')
    publisher = rospy.Publisher("/gpt_response", String, queue_size=10)
    rospy.Subscriber("/transcription", String, check_img_received)
    rospy.Subscriber("/transcription_imgs", Image, check_transcription_received)
    rospy.spin()

