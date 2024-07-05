#! /usr/bin/env python

import intera_interface
import rospy

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff,
    RobotParams,
)

class GripperController(object):

    def __init__(self,arm):
        self._arm = arm
        self._cuff = Cuff(limb=arm)
        self._gripper = get_current_gripper_interface()
        self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

        if self._is_clicksmart:
             if self._gripper.needs_init():
                  print("Clicksmart gripper needs initialization")
                  self._gripper.initialize()
        else:
             if not (self._gripper.is_calibrated() or self._gripper.calibrate() == True):
                  raise
        
        self._gripper.set_ee_signal_value('grip', True)

        rospy.signal_shutdown('Program done.')

def main():
    valid_limbs = "right"
    rospy.init_node("close_gripper")

    grip_ctrl = GripperController(valid_limbs)

    rospy.spin()

if __name__ == "__main__":
	main()