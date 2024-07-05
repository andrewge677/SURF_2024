#!/usr/bin/env python

# code copied from intera_examples

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
        rospy.init_node('go_to_joint_angles_py')
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        args = {'speed_ratio': 0.5, 
                'accel_ratio': 0.5, 
                'timeout': None,
                'joint_angles': [0.18966015625, 0.347060546875, -1.9490498046875, 1.6227490234375, 1.8296552734375, 1.9532705078125, 2.10231640625]
                }

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args['speed_ratio'],
                                         max_joint_accel=args['accel_ratio'])
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(args['joint_angles']) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = args['joint_angles'])
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args['timeout'])
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()