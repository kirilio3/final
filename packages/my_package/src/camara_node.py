#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA, Float64
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, LEDPattern
from sensor_msgs.msg import CompressedImage, CameraInfo
import signal
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge

import stage2 as Stage2
import time
import signal
import stage3 as Stage3
class Camara_node(DTROS):

    def __init__(self, node_name):
        super(Camara_node, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None


        ##############following stages are added#####################
        self.stage2 = Stage2.Stage2(self.vehicle_name,ID1=50,ID2=48)

        
        
        # # Publishers
        # self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
        
        # # Subscribers

        # self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        # self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"

        # # self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
        # self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)
        signal.signal(signal.SIGINT, self.signal_handler)
