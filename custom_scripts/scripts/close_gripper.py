#! /usr/bin/env python

import intera_interface
import rospy

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff
)

VALID_LIMB = "right"

class GripperController():
    """
    GripperController object for intializing custom gripper and closing gripper.

    Pass name of limb to use as gripper as str. If does not initialize, 
    can run open_gripper.py (even if gripper already open) to help 
    initialize when booting up
    """ 

    def __init__(self,arm):
        self._arm = arm
        self._cuff = Cuff(limb=arm)
        self._gripper = get_current_gripper_interface()
        self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

        if self._is_clicksmart:
             if self._gripper.needs_init():
                  rospy.loginfo("close_gripper.py Clicksmart gripper needs initialization")
                  self._gripper.initialize()
        else:
             if not (self._gripper.is_calibrated() or self._gripper.calibrate() == True):
                  raise
        
        self._gripper.set_ee_signal_value('grip', True)

        rospy.signal_shutdown('close_gripper.py finished init')

def main():
    rospy.init_node("close_gripper")
    rospy.loginfo("close_gripper.py entered")
    GripperController(VALID_LIMB)
    rospy.spin()
    rospy.loginfo("close_gripper.py exiting")

if __name__ == "__main__":
	main()