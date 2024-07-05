#! /usr/bin/env python

import cv2 
import rospy
# missing imports for limb, Image

def send_image(path):
        img = cv2.imread(path)
        img = cv2.resize(img,(1024,600))
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(0.1)

limb.move_to_joint_positions(joint_positions)
image_msg = rospy.wait_for_message("/cameras/left_hand_camera/image", Image)
pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
pub.publish(image_msg)
rospy.sleep(2)