#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import intera_interface
import argparse
from cv_bridge import CvBridge
import os

# realsense camera functionality does not work, also need to turn on stream before taking photo in this program

HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"			# not functional, needs conversion from mono8 encoding functionality
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

# def send_photo():
#     print()

# def take_photo(data, args):
#     try:
#         width = data.width
#         height = data.height
#         encoding = data.encoding
#         (filename, camera) = args
#         bridge = CvBridge()

#         image_data = np.frombuffer(data.data, dtype=np.uint8)

#         print("Data: {} {} {}".format(width, height, encoding))

#         if encoding == "rgb8":
#             image_array = image_data.reshape((height, width, 3))
#         elif encoding == "bgr8":
#             image_array = image_data.reshape((height, width, 3))[:,:, ::-1]
#         elif encoding == "bgra8":
#             image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
#             image_array = image_array[:,:,:3]
#         else:
#             print("encoding not recognized")

#         cv2.imwrite('/home/student/ros_ws/src/custom_scripts/images/{}.png'.format(filename), image_array)	
		
#         # rospy.init_node('gpt_image_input', anonymous=True)	
#         image_pub = rospy.Publisher('/transcription_img', Image, queue_size=10)
#         img_msg = bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
#         # print(img_msg)
#         image_pub.publish(img_msg)

#         rospy.signal_shutdown('Program end.')		

#         print("Finish callback")
#     except Exception as e:
#         rospy.logerr("Error :{0}".format(e))

# def take_photo_and_publish(filename, camera):

# 	rospy.init_node('gpt_image_input', anonymous=True)

# 	cameras = intera_interface.Cameras()	
# 	if(camera == 'head_camera'):	
# 		cameras.start_streaming(camera)	
# 		path = HEAD_CAMERA_TOPIC	
# 	else:
# 		path = REALSENSE_CAMERA_TOPIC	
	
# 	rospy.Subscriber(path, Image, take_photo, callback_args=(filename, camera))
# 	rospy.loginfo("Subscribed to topic")

# 	rospy.spin()






class TopicSynchronizer:
    def __init__(self, filename, camera):
        self.img_data = None  # Variable to store data from the first topic
        self.transcription_is_published = False  # Flag to indicate if second topic has been received
        self.filename = filename
        self.camera = camera

        rospy.init_node('transcription_imgs', anonymous=True)

        cameras = intera_interface.Cameras()	
        if(self.camera != 'rs'):	
            cameras.start_streaming(self.camera)	
            self.path = HEAD_CAMERA_TOPIC	
        else:
            self.path = REALSENSE_CAMERA_TOPIC

        # Subscribers for the two topics
        self.img_pub = rospy.Publisher("/transcription_imgs", Image, queue_size=10)
        rospy.Subscriber(self.path, Image, self.take_and_publish_img, callback_args=(filename, camera))
        rospy.Subscriber("/transcription", String, self.see_transcription)
        print("Looking for transcription publish to take photo and publish")

    def take_and_publish_img(self, data, args):

        if self.transcription_is_published:

            try:
                width = data.width
                height = data.height
                encoding = data.encoding
                (filename, camera) = args
                bridge = CvBridge()

                image_data = np.frombuffer(data.data, dtype=np.uint8)

                print("Data: {} {} {}".format(width, height, encoding))

                if encoding == "rgb8":
                    image_array = image_data.reshape((height, width, 3))
                elif encoding == "bgr8":
                    image_array = image_data.reshape((height, width, 3))[:,:, ::-1]
                elif encoding == "bgra8":
                    image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
                    image_array = image_array[:,:,:3]
                else:
                    print("encoding not recognized")

		# try:
		#     os.remove("/home/student/ros_ws/src/custom_scripts/images/{}.jpg".format(filename))
		# except: pass
		cv2.imwrite("/home/student/ros_ws/src/custom_scripts/images/{}.jpg".format(filename), image_array)
                	
                img_msg = bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
                self.img_pub.publish(img_msg)	

                print("Image taken and published after transcription seen.")
            except Exception as e:
                rospy.logerr("Error :{0}".format(e))

            self.transcription_is_published = False
        else:
            pass

    def see_transcription(self, msg):
        self.transcription_is_published = True

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            prog='take_photo_and_publish.py',
            description='Takes photo using Sawyer or attached cameras.',
            epilog='Made by Andrew Ge, SURF 2024'
        )
        
        parser.add_argument('filename', help='Name of png file to be saved')
        parser.add_argument('-c', '--camera', help='"head_camera" or "rs" for RealSense camera, default is head')
        args = parser.parse_args()
        if args.camera is None:
            args.camera = 'head_camera'	
              
        synchronizer = TopicSynchronizer(args.filename, args.camera)
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
