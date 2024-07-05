#!/usr/bin/env python
import rospy
import requests
import json
import os
from std_msgs.msg import String

# Define your OpenAI API key
API_KEY = os.getenv("OPEN_AI_API_KEY")

def chatgpt_response(prompt):
    url = "https://api.openai.com/v1/chat/completions"
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer {}".format(API_KEY)
    }
    data = { "model": "gpt-3.5-turbo", 
            "messages": [ {"role": "system", 
                           "content": "You are a helpful assistant."}, 
                           {"role": "user", "content": prompt} ], 
                           "max_tokens": 150 }
    response = requests.post(url, headers=headers, data=json.dumps(data))
    response_json = response.json()
    if 'choices' in response_json and len(response_json['choices']) > 0:
        return response_json['choices'][0]['message']["content"].strip()
    else:
        print(response)
        print(response_json)
        return "Error"

def callback(msg):
    rospy.loginfo("Received prompt: %s", msg.data)
    response = chatgpt_response(msg.data)
    response.encode('ascii', 'ignore')  # Ignore characters that cannot be encoded
    rospy.loginfo("ChatGPT response: %s", response)
    pub.publish(response)

def main():
    global pub
    rospy.init_node('chatgpt_node', anonymous=True)
    rospy.Subscriber('/transcription', String, callback)
    pub = rospy.Publisher('/chatgpt_response', String, queue_size=10)

    rospy.loginfo("ChatGPT Node has started. Listening to /chatgpt_prompt...")
    rospy.spin()

if __name__ == '__main__':
    main()