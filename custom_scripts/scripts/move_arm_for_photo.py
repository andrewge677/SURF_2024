#!/usr/bin/env python

# Code modified from intera_examples

import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

def main():

    try:
        rospy.init_node('move_arm_for_photo')
        rospy.loginfo("move_arm_for_photo.py entered")
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        args = {'speed_ratio': 0.5, 
                'accel_ratio': 0.5, 
                'timeout': None,
                'joint_angles': [1.466916015625, -0.0958193359375, -1.5692177734375, 1.5055166015625, 1.448626953125, 1.6323173828125, 3.2174990234375]
                }

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args['speed_ratio'],
                                         max_joint_accel=args['accel_ratio'])
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(args['joint_angles']) != len(joint_angles):
            rospy.logerr('move_arm_for_photo.py The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = args['joint_angles'])
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args['timeout'])
        if result is None:
            rospy.logerr('move_arm_for_photo.py Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('move_arm_for_photo.py move_arm_for_photo.py successfully moved arm for photo')
        else:
            rospy.logerr('move_arm_for_photo.py Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('move_arm_for_photo.py Keyboard interrupt detected from the user. Exiting before trajectory completion.')

    rospy.loginfo("move_arm_for_photo.py exiting")


if __name__ == '__main__':
    main()