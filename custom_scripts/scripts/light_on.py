#!/usr/bin/env python

import rospy
from intera_interface import Lights

def turn_on_light():
    rospy.init_node("turn_on_light")
    lights = Lights()
    light_name = "head_red_light"
    lights.set_light_state(light_name, on=True)
    rospy.loginfo("light_on.py turning on light")

if __name__ == '__main__':
    turn_on_light()