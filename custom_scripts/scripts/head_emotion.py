#! /usr/bin/env python

# does not work yet, Sheldon is working 

import intera_interface
import argparse
import rospy

def feeling(name):
    print("Feels: {}".format(name))

if __name__ == "__main__":
    feeling("World")