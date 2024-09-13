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
import inspect

HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"


class PhotoError(Exception):
    def __init__(self, message="An error occurred while taking the photo."):
        self.message = message
        super(PhotoError, self).__init__(self.message)

class PhotoTakerPublisher:
    """
    Object listens to /transcription topic, and once sees publish, takes photo to accompany
    transcription text in GPT API request.
    """
    def __init__(self, filename, camera):
        self.img_data = None 
        self.transcription_is_published = False  
        self.filename = filename
        self.camera = camera

        rospy.init_node('transcription_imgs', anonymous=True)
        rospy.loginfo("take_photo_and_publish.py entered")
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
        rospy.loginfo("take_photo_and_publish.py Looking for transcription publish to take photo and publish")

    def take_and_publish_img(self, data, args):
        file_name = inspect.getfile(inspect.currentframe())
        error_pub = rospy.Publisher('/photo_errors', String, queue_size=10)

        if self.transcription_is_published:
            try:
                width = data.width
                height = data.height
                encoding = data.encoding
                (filename, camera) = args
                bridge = CvBridge()

                image_data = np.frombuffer(data.data, dtype=np.uint8)

                rospy.loginfo("take_photo_and_publish.py photo taken")

                if encoding == "rgb8":
                    image_array = image_data.reshape((height, width, 3))[:,:, ::-1]
                elif encoding == "bgr8":
                    image_array = image_data.reshape((height, width, 3))
                elif encoding == "bgra8":
                    image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
                    image_array = image_array[:,:,:3]
                else:
                    rospy.loginfo("take_photo_and_publish.py encoding not recognized")

                cv2.imwrite("/home/student/ros_ws/src/custom_scripts/images/{}.jpg".format(filename), image_array)
                
                # if colors are wrong, switch encoding to 'rgb8'
                img_msg = bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
                self.img_pub.publish(img_msg)    

                rospy.loginfo("take_photo_and_publish.py image published")
            except Exception as e:
                error_message = "PhotoError in {}: {}".format(file_name, e)
                rospy.logerr(error_message)
                error_pub.publish(error_message)

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
              
        synchronizer = PhotoTakerPublisher(args.filename, args.camera)
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException as e:
        file_name = inspect.getfile(inspect.currentframe())
        rospy.logerr("ROSInterruptException in {}: {}".format(file_name, e))
    rospy.loginfo("take_photo_and_publish.py exiting")