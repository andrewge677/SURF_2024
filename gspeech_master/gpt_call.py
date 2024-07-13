#!/usr/bin/env python3

from openai import OpenAI
import base64

API_KEY = os.getenv("OPEN_AI_API_KEY")
MODEL = 'gpt-4o'

TRANSCRIPTION_PATH = "/home/student/ros_ws/src/custom_scripts/transcripts/transcription.txt"
IMAGE_PATH = "/home/student/ros_ws/src/custom_scripts/images/transcription_img.jpg"

def encode_image(image_path):
    with open(image_path, 'rb') as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def api_request():
	base64_image = encode_image(IMAGE_PATH)
	transcription = ""
	with open(TRANSCRIPTION_PATH, 'r') as f:
	    transcription = f.read()
	    
	response = client.chat.completions

	response = client.chat.completions.create(
	    model=MODEL,
	    messages=[
		{"role": "system", "content": "You are a helpful assistant."},
		{"role": "user", "content": [
		    {"type": "text", "text": transcription},
		    {"type": "image_url", "image_url": {
		        "url": f"data:image/png;base64,{base64_image}"}
		    },
		]}
	    ],
	    temperature=0.0,
	)
	print(response.choices[0].message.content)

if __name__ == "__main__":
    api_request()
