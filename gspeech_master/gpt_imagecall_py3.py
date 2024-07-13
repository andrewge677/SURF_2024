# !/usr/bin/env python3

# to run this need to call start virtual environment that uses python 3.7.5
# run 'source openaienv/bin/activate
# then add API key 

import openai
# from openai import OpenAI
import base64
import os

API_KEY = os.getenv("OPEN_AI_API_KEY")
MODEL = 'gpt-4o'

openai.api_key = API_KEY
# client = OpenAI(api_key = API_KEY)

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
		
	response = openai.chat.completions.create(
		model=MODEL,
		# messages=[
		# 	{"role": "system", "content": "You are a helpful assistant."},
		# 	{"role": "user", "content": 
    	# 		[
		# 			{"type": "text", "text": transcription},
		# 			{"type": "image_url", "image_url": 
      	# 				{"url": f"data:image/jpeg;base64,{base64_image}"}
		# 			},
		# 		]
		# 	}
		# ]
		# temperature=0.0,
        messages=[
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
	],
	)
	print(response.choices[0].message.content)
     

if __name__ == "__main__":
    api_request()



