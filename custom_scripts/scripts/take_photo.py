#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import intera_interface
import argparse

# realsense camera functionality does not work, also need to turn on stream before taking photo in this program

HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"			# not functional, needs conversion from mono8 encoding functionality
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

def callback(data, args):
	try:
		width = data.width
		height = data.height
		encoding = data.encoding
		(filename, camera) = args

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

		cv2.imwrite('/home/student/ros_ws/src/custom_scripts/images/{}.jpg'.format(filename), image_array)		

		rospy.signal_shutdown('Program end.')		

		print("Finish callback")
	except Exception as e:
		rospy.logerr("Error :{0}".format(e))

def take_photo(filename, camera):

	rospy.init_node('take_photo', anonymous=True)

	cameras = intera_interface.Cameras()	
	if(camera == 'head_camera'):	
		cameras.start_streaming(camera)	
		path = HEAD_CAMERA_TOPIC	
	else:
		path = REALSENSE_CAMERA_TOPIC	
	
	rospy.Subscriber(path, Image, callback, callback_args=(filename, camera))
	rospy.loginfo("Subscribed to topic")

	rospy.spin()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(
		prog='take_photo.py',
		description='Takes photo using Sawyer.',
		epilog='Made by Andrew Ge, SURF 2024'
	)
	
	parser.add_argument('filename', help='Name of png file to be saved')
	parser.add_argument('-c', '--camera', help='"head_camera" or "rs" for RealSense camera, default is head')
	args = parser.parse_args()
	if args.camera is None:
		args.camera = 'head_camera'			

	take_photo(args.filename, args.camera)
