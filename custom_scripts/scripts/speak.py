#! /usr/bin/env python

import os
import rospy
import message_filters
import json
import random
import pickle
import pyttsx3
import copy

#from rl_experiment.srv import UserLangInputService
from std_srvs.srv import Trigger, TriggerResponse

from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import cv_bridge
import cv2

def speak_sentence(sentence):
        rospy.loginfo("Baxter says: %s", sentence) 
        soundhandle = SoundClient()
        rospy.sleep(0.1)
        soundhandle.say(clean_text(sentence))

def SpeakSentence(sentence):
        engine = pyttsx3.init()
        voices = engine.getProperty('voices')
        #engine.setProperty('voice', voices[16].name) #english-us
        engine.setProperty('voice', 'english-us')
        engine.setProperty('rate', 150)
        engine.say(sentence)
        engine.runAndWait()