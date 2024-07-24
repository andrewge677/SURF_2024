#!/usr/bin/env python

# code copied from intera_examples

import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

def goto_table_neutral():
    """
    goto_table_neutral primes arm to execute goto_square
    """
    try:
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
            rospy.logerr('goto_table_neutral.py The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = args['joint_angles'])
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args['timeout'])
        if result is None:
            rospy.logerr('goto_table_neutral.py Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('goto_table_neutral.py went to table neutral position successfully')
        else:
            rospy.logerr('goto_table_neutral.py Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('goto_table_neutral.py Keyboard interrupt detected from the user. Exiting before trajectory completion.')

def main():
    rospy.init_node('goto_table_neutral')
    rospy.loginfo("goto_table_neutral.py entered")
    goto_table_neutral()
    rospy.loginfo("goto_table_neutral.py exiting")

if __name__ == '__main__':
    main()