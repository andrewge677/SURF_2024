#!/usr/bin/env python2

"""import cv2
import numpy as np
from apriltag import Detector

def find_apriltag(image_path):
    # Load image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Error: Unable to read image from {}".format(image_path))
        return

    # Create detector
    detector = Detector()

    # Detect tags in the image
    detections = detector.detect(img)

    if detections:
        print("Number of tags detected: {}".format(len(detections)))
        for i, detection in enumerate(detections):
            # Print tag id and pose information
            print ("Tag ID: {}".format(detection.tag_id))
            print ("Tag Family: {}".format(detection.tag_family.decode('utf-8')))

            # Compute pose information
            camera_params = [img.shape[1], img.shape[0], 520.0, 520.0]  # Example camera parameters (width, height, fx, fy)
            tag_size = 0.16  # Example tag size in meters
            pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)

            # Extract position (XYZ) and orientation (quaternion)
            xyz = pose[:3, 3]  # XYZ coordinates
            quaternion = pose[:3, :3]  # Rotation matrix

            # Convert rotation matrix to quaternion (if needed)
            # Assuming the library returns a rotation matrix
            rotation_matrix = quaternion
            qw = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2.0
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * qw)
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * qw)
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * qw)

            # Print position and quaternion
            print( "Tag Position (XYZ): {}".format(xyz))
            print ("Tag Orientation (Quaternion): ({}, {}, {}, {})".format(qx, qy, qz, qw))
            print ("")

            # Draw tag outline
            for idx in range(len(detection.corners)):
                cv2.line(img, tuple(detection.corners[idx - 1, :].astype(int)),
                         tuple(detection.corners[idx, :].astype(int)), (0, 255, 0), 5)

    else:
        print ("No AprilTags were detected.")

    # Display the image with tag outlines
    cv2.imshow('AprilTag Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = '/home/student/ros_ws/src/custom_scripts/images/testing_tags/flat2.jpg'  # Replace with your image path
    find_apriltag(image_path)"""

import cv2
from apriltag import Detector

def find_apriltag(image_path):
    # Load image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Error: Unable to read image from {}".format(image_path))
        return

    # Create detector
    detector = Detector()

    # Detect tags in the image
    detections = detector.detect(img)

    if detections:
        print("Number of tags detected: {}".format(len(detections)))
        for i, detection in enumerate(detections):
            # Print tag id and pose information
            print("Tag ID: {}".format(detection.tag_id))
            print("Tag Family: {}".format(detection.tag_family.decode('utf-8')))

            # Compute pose information
            camera_params = [img.shape[1], img.shape[0], 520.0, 520.0]  # Example camera parameters (width, height, fx, fy)
            tag_size = 0.16  # Example tag size in meters
            pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)

            # Extract position (XYZ) and orientation (quaternion)
            xyz = pose[:3, 3]  # XYZ coordinates
            quaternion = pose[:3, :3]  # Rotation matrix

            # Print position and quaternion
            print("Tag Position (XYZ): {}".format(xyz))
            print("Tag Orientation (Quaternion): {}".format(quaternion.flatten()))
            print("")

            # Draw tag outline
            cv2.polylines(img, [np.int32(detection.corners)], True, (0, 255, 0), 2)

    else:
        print("No AprilTags were detected.")

    # Display the image with tag outlines
    cv2.imshow('AprilTag Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = '/home/student/ros_ws/src/custom_scripts/images/testing_tags/flat2.jpg'  # Replace with your image path
    find_apriltag(image_path)
