#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA, Float64,Int64
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
from moduls.detect_apriltag import DetectApriltag as da
from moduls.detect_cross import DetectCorss as dc
class Camara_node(DTROS):

    def __init__(self, node_name):
        super(Camara_node, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None
        # self.DA = da(self.vehicle_name)
        self.DC = dc(self.vehicle_name, None,None, debugger=False
                     
                     ,corss_or_red="red")

        ##############following stages are added#####################


        
        
        # # Publishers
        # self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
        
        # # Subscribers

        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"

        # # self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
        self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)
        self.signal_sub = rospy.Subscriber(f"/{self.vehicle_name}/send_of_sig", Int64, self.signal_cb)


    def cb_camera_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.D)
        self.DC.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
        # self.DA.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
    
    def signal_cb(self, msg):
        print(msg.data)
        if msg.data == 1:
            # print("signal received")
            super(Camara_node, self).on_shutdown()
            sys.exit(0)


    def run(self):
        rospy.spin()

    def on_shutdown(self):
        # self.stage2.stop()
        super(Camara_node, self).on_shutdown()
        sys.exit(0)

    # def signal_handler(self, sig, frame):
    #     rospy.loginfo("Ctrl+C detected, shutting down...")
    #     self.on_shutdown()
    #     sys.exit(0)

    
if __name__ == '__main__':

    node = Camara_node(node_name="main")
    node.run()
