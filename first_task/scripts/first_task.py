#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2

# cv2 and cv_bridge

def callback(data):
	try:
		print("Callback entered")

		width = data.width
		height = data.height
		encoding = data.encoding

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

		with open('/home/student/ros_ws/src/first_task/images/image.ppm', 'wb') as f:
			f.write(bytearray("P6\n{} {}\n255\n".format(width, height), 'ascii'))
			f.write(image_array.tobytes())

		print("Finish callback")
	except Exception as e:
		rospy.logerr("Error :{0}".format(e))

def first_task():
	
	rospy.init_node('first_task', anonymous=True)
	
	rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, callback)
	rospy.loginfo("Subscribed to topic")

	rospy.spin()

if __name__ == "__main__":
	first_task()
