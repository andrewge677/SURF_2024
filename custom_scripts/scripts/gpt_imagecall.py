#!/usr/bin/env python

import requests
import json
import cv2
import base64
import os

API_KEY = os.getenv("OPEN_AI_API_KEY")

# Define your OpenAI API key and endpoint
API_URL = 'https://api.openai.com/v1/chat/completions'

def gpt4_vision_response(image_path,prompt):
    url = API_URL
    headers = {
        "Authorization": "Bearer {}".format(API_KEY),
        "Content-Type": "application/json"
    }
    
    # Read image from file
    image = cv2.imread(image_path)
    max_dim = 512  # Maximum dimension for resizing 
    height, width = image.shape[:2] 
    if max(height, width) > max_dim: 
        scale = max_dim / float(max(height, width)) 
        new_size = (int(width * scale), int(height * scale)) 
        image = cv2.resize(image, new_size)

    _, img_encoded = cv2.imencode('.jpg', image)
    img_base64 = base64.b64encode(img_encoded).decode('utf-8')

    data = { "model": "gpt-4-turbo", 
            "messages": [ {"role": "system", "content": "You are a helpful assistant."}, 
            {"role": "user", "content": prompt}, 
            {"role": "user", "content": "<img src='data:image/jpeg;base64,{}'>".format(img_base64)} ],"max_tokens": 150 }
    
    response = requests.post(url, headers=headers, data=json.dumps(data))
    response_json = response.json()
    if 'choices' in response_json and len(response_json['choices']) > 0:
        return response_json['choices'][0]['message']['content'].strip()
    else:
        print(response)
        print(response_json)
        return "Error processing image"

if __name__ == '__main__':
    # Path to the image file
    image_path = '/home/student/Downloads/earth.jpg'
    prompt = "Describe the contents of the image"
    
    response_text = gpt4_vision_response(image_path,prompt)
    
    if response_text:
        print("Response: {}".format(response_text))


