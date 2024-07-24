#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageCapture:
    """
    Takes photo using realsense camera.
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_received = False

    def image_callback(self, msg):
        try:
            if not self.image_received:
                # Convert the image to a cv2 image
                cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                self.image_received = True
                # Save the image in its original format
                cv2.imwrite("/home/student/ros_ws/src/custom_scripts/images/rs_captured_image.png", cv_image)  # Save the image to images folder
                rospy.loginfo("Image captured and saved as rs_captured_image.png")
                rospy.signal_shutdown("Image captured, shutting down.")  # Shutdown ROS node
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        except Exception as ex:
            rospy.logerr("Exception: %s", ex)

def main():
    rospy.init_node('image_capture', anonymous=True)
    rospy.loginfo("realsense_image.py entered")

    # Ensure the ROS node waits for the image topic to be available
    rospy.loginfo("realsense_image.py Waiting for image topic...")

    try:
        rospy.wait_for_message('/camera/color/image_raw', Image, timeout=10)
        rospy.loginfo("realsense_image.py Image topic detected. Capturing image...")
    except rospy.ROSException as e:
        rospy.logerr("realsense_image.py Timeout waiting for image topic. Ensure the topic is being published: %s", e)
        rospy.signal_shutdown("Timeout waiting for image topic")

    rospy.spin()
    rospy.loginfo("realsense_image.py exiting")

if __name__ == '__main__':
    main()
