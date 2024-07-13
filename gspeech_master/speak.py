#! /usr/bin/env python3

import pyttsx3

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
        engine.say(sentence)
        engine.runAndWait()



SpeakSentence("Hello there")