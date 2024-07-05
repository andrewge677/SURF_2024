#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_received = False

    def image_callback(self, msg):
        if not self.image_received:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.image_received = True
                self.image_sub.unregister()  # Unsubscribe from the topic
                cv2.imwrite("/home/student/ros_ws/src/custom_scripts/images/rs_captured_image.jpg", cv_image)  # Save the image to images folder
                rospy.loginfo("Image captured and saved as realsense_captured_image.jpg")
            except CvBridgeError as e:
                rospy.logerr(e)
            except Exception as ex:
                rospy.logerr(ex)

def main():
    rospy.init_node('image_capture', anonymous=True)
    image_capture = ImageCapture()
    rospy.spin()

if __name__ == '__main__':
    main()
