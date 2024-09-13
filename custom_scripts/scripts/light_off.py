#!/usr/bin/env python

import rospy
from intera_interface import Lights

def turn_off_light():
    rospy.init_node("turn_off_light")
    lights = Lights()
    light_name = "head_red_light"
    lights.set_light_state(light_name, on=False)
    rospy.loginfo("light_off.py turning off light")

if __name__ == '__main__':
    turn_off_light()