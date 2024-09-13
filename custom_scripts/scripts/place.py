#!/usr/bin/env python

import argparse
import rospy
import argparse
import subprocess
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb


PREP_POSITION = [       # 12 squares with 7 joint angles each
    [1.2103203125, 0.213666015625, -1.0019970703125,
        1.597095703125, 1.83233984375, 1.015109375, 2.80941015625],
    [0.8912587890625, 1.0423818359375, -1.30570703125, 1.9660537109375,
     2.5717109375, 1.8184150390625, 2.252509765625],
    [0.3651708984375, 0.878166015625, -1.1147626953125, 2.0045927734375,
     2.5311318359375, 1.55193359375, 1.5363525390625],
    [-0.4260537109375, 0.8128896484375, -1.456416015625, 1.6018779296875,
     2.4136943359375, 1.4557861328125, 1.252220703125],
    [0.59241796875, 0.9057578125, -2.05596875, 1.2782978515625,
     2.5439013671875, 1.64681640625, 2.9164287109375],
    [0.4317978515625, 1.078619140625, -1.7984306640625, 1.6688408203125,
     2.6443994140625, 1.7579921875, 2.2975810546875],
    [0.076755859375, 0.927478515625, -1.563009765625, 1.5769638671875, 2.5179501953125,
     1.527080078125, 1.8005234375],                        
    [-0.574828125, 0.691302734375, -1.6605166015625, 1.112564453125,
     2.25557421875, 1.38238671875, 1.64798046875],
    [0.274193359375, 0.607818359375, -2.03475390625, 0.5802646484375,
     2.2943330078125, 1.30192578125, 3.0262880859375],
    [0.1450751953125, 0.7319716796875, -1.9372294921875,
     1.046080078125, 2.3658720703125, 1.4080927734375, 2.5130390625],
    [-0.2275625, 0.699439453125, -1.84805078125, 0.986408203125,
     2.316810546875, 1.3575078125, 2.1596796875],
    [-0.8537294921875, 0.5310166015625, -1.9386640625,
     0.3382197265625, 2.11775, 1.154818359375, 2.10502734375]
]
PLACE_POSITION = [       # 12 squares with 7 joint angles each
    [1.1646865234375, 0.264515625, -0.82244921875, 1.53290234375,
        1.971998046875, 0.9239580078125, 2.61269921875],
    [0.89208203125, 1.142859375, -1.2847998046875, 1.8781025390625,
     2.6586279296875, 1.7677265625, 2.252509765625],
    [0.408900390625, 0.9549677734375, -1.0726591796875,
     1.892296875, 2.6137236328125, 1.4841767578125, 1.6313125],
    [-0.40237890625, 0.926140625, -1.4593037109375, 1.561453125,
     2.5338173828125, 1.3850849609375, 1.2611552734375],
    [0.57415234375, 0.95414453125, -2.0556796875, 1.2355537109375,
     2.603626953125, 1.581990234375, 2.9166484375],
    [0.4010986328125, 1.09622265625, -1.817181640625, 1.5768193359375,
     2.622580078125, 1.7066708984375, 2.3586884765625],
    [0.02752734375, 0.9686611328125, -1.5813271484375, 1.5310263671875,
     2.555650390625, 1.5784658203125, 1.8007294921875],
    [-0.576783203125, 0.7351611328125, -1.6761005859375,
     1.0733134765625, 2.264224609375, 1.31478515625, 1.64798046875],
    [0.2725458984375, 0.653419921875, -2.03475390625, 0.567830078125,
     2.3392880859375, 1.2093935546875, 3.0262880859375],
    [0.14744140625, 0.8077373046875, -1.942408203125,
     1.02594921875, 2.43409375, 1.3282646484375, 2.5130390625],
    [-0.2683017578125, 0.7461728515625, -1.8562802734375,
     0.92471484375, 2.3768076171875, 1.35087109375, 2.155302734375],
    [-0.8575439453125, 0.5782705078125, -1.9395302734375,
     0.3382197265625, 2.1753193359375, 1.075119140625, 2.1046142578125]
]


def place(square_num):
    """
    Takes a square number 1-12 inclusive and places the object at that location.
    Does not provide checking to see if object is held, or if space is empty.
    """

    square_num = int(square_num)

    try:
        rospy.init_node('place_py')
        rospy.loginfo("place.py entered")
        limb = Limb()
        down_traj = MotionTrajectory(limb=limb)

        prep_args = {'speed_ratio': 0.5,
                     'accel_ratio': 0.5,
                     'timeout': None,
                     'joint_angles': PREP_POSITION[square_num - 1]
                     }

        grab_args = {'speed_ratio': 0.5,
                     'accel_ratio': 0.5,
                     'timeout': None,
                     'joint_angles': PLACE_POSITION[square_num - 1]
                     }

        # map prep waypoints
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=prep_args['speed_ratio'],
                                         max_joint_accel=prep_args['accel_ratio'])
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles=joint_angles)
        down_traj.append_waypoint(waypoint.to_msg())

        if len(prep_args['joint_angles']) != len(joint_angles):
            rospy.logerr(
                'place.py  The number of preparation joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles=prep_args['joint_angles'])
        down_traj.append_waypoint(waypoint.to_msg())

        # map grab waypoints
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=grab_args['speed_ratio'],
                                         max_joint_accel=grab_args['accel_ratio'])
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

        joint_angles = limb.joint_ordered_angles()

        if len(grab_args['joint_angles']) != len(joint_angles):
            rospy.logerr(
                'place.py  The number of place joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles=grab_args['joint_angles'])
        down_traj.append_waypoint(waypoint.to_msg())

        # send trajectory
        result = down_traj.send_trajectory(timeout=grab_args['timeout'])
        if result is None:
            rospy.logerr('place.py  Downwards place trajectory FAILED to send')
            return

        # results
        if not result.result:
            rospy.logerr('place.py  Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        # place object
        try:
            subprocess.check_output(['python', '/home/student/ros_ws/src/custom_scripts/scripts/open_gripper.py'])
        except subprocess.CalledProcessError as e:
            rospy.logerr("place.py  Error calling open_gripper.py from place.py: {}".format(e))

        # move to prep location
        up_traj = MotionTrajectory(limb=limb)

        waypoint.set_joint_angles(joint_angles=prep_args['joint_angles'])
        up_traj.append_waypoint(waypoint.to_msg())

        # send trajectory
        result = up_traj.send_trajectory(timeout=prep_args['timeout'])
        if result is None:
            rospy.logerr('place.py  Up trajectory FAILED to send')
            return

        # move to neutral
        try:
            subprocess.check_output(['python', '/home/student/ros_ws/src/custom_scripts/scripts/goto_table_neutral.py'])
        except subprocess.CalledProcessError as e:
            rospy.logerr("place.py  Error calling goto_table_neutral.py from place.py: {}".format(e))

        if result.result:
            rospy.loginfo("place.py finished successfully.")
        else:
            rospy.logerr('place.py  Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

    except rospy.ROSInterruptException:
        rospy.logerr(
            'place.py Keyboard interrupt detected from the user. Exiting before trajectory completion.')
        
    rospy.loginfo("place.py exiting")
        

if __name__ == "__main__":

    parser = argparse.ArgumentParser(prog='place.py',
                                     description='place item from grid square. ',
                                     epilog='Made by Andrew Ge, SURF 2024')

    parser.add_argument(
        'square_num', help="place held item at square numbered 1 through 12, inclusive, does not contain error checking for not holding item")

    args = parser.parse_args()
    place(args.square_num)