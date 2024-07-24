# !/usr/bin/env python3

import openai
import base64
import os
import json

API_KEY = os.getenv("OPEN_AI_API_KEY")
MODEL = 'gpt-4o'

openai.api_key = API_KEY

TRANSCRIPTION_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/transcription.txt"
IMAGE_PATH = "/home/student/ros_ws/src/custom_scripts/images/transcription_img.jpg"
STARTUP_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/startup.json"
LOG_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/log.json"

def encode_image(image_path):
    with open(image_path, 'rb') as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def api_request():
    messages = [{'error':'messages loaded incorrectly'}]
    base64_image = encode_image(IMAGE_PATH)
    transcription = ""
    with open(TRANSCRIPTION_PATH, 'r') as f:
        transcription = f.read()
          
    with open(LOG_PATH, 'rb') as log:
        messages = json.load(log)
        
    new_message = messages + [
        {
            "role": "user",
            "content": [
                {"type": "text", "text": transcription},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}",
                    },
                },
            ],
        }
    ]
      
    messages.append(
        {
            "role": "user",
            "content": transcription
        }
    )
     
    with open(LOG_PATH, "w") as log:
          json.dump(messages, log)
          
    response = openai.chat.completions.create(
        model=MODEL,
        messages=new_message
    )
    print(response.choices[0].message.content)
     

if __name__ == "__main__":
    api_request()



