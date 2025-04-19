#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import ColorRGBA, Float64, Int64
import cv2
from cv_bridge import CvBridge
import numpy as np
from dt_apriltags import Detector
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from mother_of_all import MotherOfAll as MOA

class DetectApriltag(MOA):

    def __init__(self, vehicle_name):

        # static parameters
        self._vehicle_name = vehicle_name
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        self._undistorted_topic = f"/{self._vehicle_name}/camera_node/image/distorted_image/compressed"

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # variables to store camera matrix and distortion coefficients
        self._camera_matrix = None
        self._distortion_coeffs = None

        # Initialize dt_apriltags Detector
        self._detector = Detector(
            families='tag36h11',  # Duckietown uses tag36h11 (Part 1.3.e)
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            searchpath=['apriltags'],
            debug=0
        )


        
        # construct subscriber for image topics
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        
        # Publisher for the processed image with AprilTag detections
        self.image_pub = rospy.Publisher(self._undistorted_topic, CompressedImage, queue_size=10)
        self.tag_id  = rospy.Publisher(f"/{self._vehicle_name}/tag_id", Int64, queue_size=10)

    def camera_info_setter(self, camera_matrix, distortion_coeffs):
        self._camera_matrix = camera_matrix
        self._distortion_coeffs = distortion_coeffs

    def callback(self, msg):
        if self._camera_matrix is None or self._distortion_coeffs is None:
            rospy.logwarn("Waiting for camera calibration parameters...")
            return
        
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Undistort the image using the camera calibration parameters
        undistorted_image = cv2.undistort(image, self._camera_matrix, self._distortion_coeffs)
        undistorted_image = cv2.GaussianBlur(undistorted_image, (9, 9), 0)
        height, width = undistorted_image.shape[:2]
        crop_top = int(height // 4)  # Start from halfway down (240 for 480 height)
        
        crop_bottom = height    # Go to the bottom (480)
        crop_left = 0           # Start from the left edge
        crop_right = width      # Go to the right edge (640)
        undistorted_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
        
        # Convert to black and white (grayscale)
        bw_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        # Extract camera parameters for dt_apriltags
        fx = self._camera_matrix[0, 0]  # focal length x
        fy = self._camera_matrix[1, 1]  # focal length y

        cx = self._camera_matrix[0, 2]  # optical center x
        cy = self._camera_matrix[1, 2]  # optical center y
        camera_params = (fx, fy, cx, cy)

        ############################ Part 1.3.a ############################
        # Detect AprilTags in the grayscale image 
        tags = self._detector.detect(
            bw_image,
            estimate_tag_pose=True,  # Enable pose estimation
            camera_params=camera_params,
            tag_size=0.065  # Duckietown tags are typically 6.5cm, adjust if different
        )
        ####################################################################

        ############################ Part 1.3.b ############################
        # Draw bounding boxes and tag IDs on the image
        # Draw bounding boxes and tag IDs on the image
        output_image = cv2.cvtColor(bw_image, cv2.COLOR_GRAY2BGR)  # Convert to BGR for colored drawings
        for tag in tags:
            # self.tag_id = tag.tag_id  # Store the detected tag ID
            self.tag_id.publish(Int64(tag.tag_id))  # Publish the tag ID
            # print(tag.tag_id)
            # Draw bounding box
            # for i in range(4):
            #     pt1 = (int(tag.corners[i-1][0]), int(tag.corners[i-1][1]))
            #     pt2 = (int(tag.corners[i][0]), int(tag.corners[i][1]))
            #     cv2.line(output_image, pt1, pt2, (0, 255, 0), 2)
        ####################################################################

        ############################ Part 1.3.c ############################
            # Draw tag ID
            # tag_center = (int(tag.center[0]), int(tag.center[1]))  # Use tag.center for center position
            # cv2.putText(output_image, str(tag.tag_id), tag_center, 
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        ####################################################################

        ############################ Part 1.3.d ############################

        # # Convert the processed image back to a ROS CompressedImage message
        # processed_msg = self._bridge.cv2_to_compressed_imgmsg(output_image)

        # # Publish the processed image with AprilTag detections
        # self.image_pub.publish(processed_msg)


    # def get_tag_id(self):
    #     return self.tag_id
