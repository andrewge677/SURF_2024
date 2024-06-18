#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from intera_interface import Limb, RobotEnable
from intera_interface import CHECK_VERSION
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped 
import numpy as np

# Global variable to store the image data
image_data = None

def move_to_cartesian_pose(pose):
    rospy.init_node('move_sawyer_arm_to_pose')

    limb = Limb()

    ik_service_name = "/ExternalTools/right/PositionKinematicsNode/IKService"
    rospy.wait_for_service(ik_service_name)
    ik_service = rospy.ServiceProxy(ik_service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()

    ik_request.pose_stamp.append(pose)
    ik_request.tip_names.append('right_hand')
    
    try:
        ik_response = ik_service(ik_request)
        if ik_response.result_type[0] > 0:
            # Extract the joint angles from the IK response
            joint_angles = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            rospy.loginfo("IK Solution found:")
            rospy.loginfo(joint_angles)

            # Move the arm to the computed joint angles
            limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("IK solution not found")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {0}".format(e))

# Callback function for the camera subscriber
def image_callback(msg):
    global image_data
    # Convert the ROS Image message to a CV2 image (in BGR8 format)
    bridge = CvBridge()
    image_data = bridge.imgmsg_to_cv2(msg, "bgra8")             # does bgra8 work here

# Function to save the image data to a file
def save_image(filename):
    global image_data
    if image_data is not None:
        cv2.imwrite(filename, image_data)
    else:
        print("No image data to save.")

# Function to move the camera
def move_camera():
    # Get the current joint angles
    limb = Limb()
    joint_angles = limb.joint_angles()

    # Modify the joint angles to move the camera
    # NOTE: Replace this with the actual joint angles for your setup
    joint_angles['right_j6'] += 0.1

    # Move the arm
    limb.move_to_joint_positions(joint_angles)

def main():
    # Initialize the ROS node
    rospy.init_node('camera_control')

    # move_to _cartesian_pose()

    # Subscribe to the camera image topic
    rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, image_callback)

    # Wait for image data
    rospy.sleep(1)

    # Save the initial image
    save_image('/home/student/ros_ws/src/first_task/images/image1.png')

    # Move the camera
    move_camera()

    # Wait for the camera to move and for new image data
    rospy.sleep(1)

    # Save the new image
    save_image('/home/student/ros_ws/src/first_task/images/image2.png')

# if __name__ == '__main__':
#     # Define the desired Cartesian pose
#     pose = PoseStamped()
#     pose.header.frame_id = "base"
#     pose.pose.position.x = 0.6
#     pose.pose.position.y = 0.0
#     pose.pose.position.z = 0.4
#     pose.pose.orientation.x = 0.0
#     pose.pose.orientation.y = 1.0
#     pose.pose.orientation.z = 0.0
#     pose.pose.orientation.w = 0.0

#     move_to_cartesian_pose(pose)

if __name__ == '__main__':
    main()