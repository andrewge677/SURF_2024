#! /usr/bin/env python

import pyttsx3
import rospy

#from rl_experiment.srv import UserLangInputService
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int32, String

def SpeakSentence(sentence):
        engine = pyttsx3.init()
        voices = engine.getProperty('voices')

        # for i, voice in enumerate(voices):
        #         print(f"Voice {i}: {voice.name}, ID: {voice.id}")


        engine.setProperty('voice', voices[16].name) #english-us
        #engine.setProperty('voice', 'english-us')
        engine.setProperty('rate', 150)
        engine.say(sentence.data)
        engine.runAndWait()



def speak_output():
    rospy.init_node('speak_output')
    rospy.Subscriber("/chatgpt_response", String, SpeakSentence)
    rospy.spin()


if __name__ == '__main__':
      speak_output()