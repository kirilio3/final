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
import moduls.lane_following as lf
import moduls.detect_cross as dc
import moduls.detect_apriltag as da
import time
import signal
from stage2 import Stage2 as S2

class Stage3(S2):

    def __init__(self, vehicle_name,ID1,ID2):
        super(Stage3, self).__init__(vehicle_name,ID1,ID2)
        self.vehicle_name = vehicle_name

        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.cross_walk_number =0
        ##############following functiinalities are added#####################
        self.LF = lf.Lane_Following(self.vehicle_name)
        self.DC = dc.Detect_Corss(self.vehicle_name, None,None, debugger=False,corss_or_red="cross")
        # self.DCR = dc.Detect_Corss(self.vehicle_name, None,None, debugger=False,corss_or_red="red")
        # self.DA = da.DetectApriltag(self.vehicle_name)

        self.distance = 10
        
        
        # Publishers
        self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
        self.red_cross_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/red_cross_line_detect", Float64, queue_size=1)
        
        # Subscribers

        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"

        # self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
        self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)

    '''
        please set camera info here for every camera related node 
    '''



    def stop(self):
        self.LF.stop()
    

    def run_stage3(self):
        rospy.sleep(1)
        rate = 20
        end = True
        while end:  
            while not rospy.is_shutdown(): 
                self.LF.lane_follow(10,rate)
                if self.cross_walk_number < 3:
                    try:
                        cross_detected = rospy.wait_for_message(f"/{self.vehicle_name}/corss_line_detect",Float64, timeout=1).data
                    except rospy.ROSException:
                        cross_detected = False
                    if cross_detected:
                        self.cross_walk_number+=1
                        rospy.loginfo("cross line detected")
                        rospy.sleep(1)
                        self.LF.stop()
                        try:
                            ped_detected = rospy.wait_for_message(f"/{self.vehicle_name}/pedestrian_detect",Float64, timeout=5).data
                        except rospy.ROSException:
                            ped_detected = True
                        while ped_detected:
                            try:
                                ped_detected = rospy.wait_for_message(f"/{self.vehicle_name}/pedestrian_detect",Float64, timeout=5).data
                            except rospy.ROSException:
                                ped_detected = True
                else:
                    self.DC.cross_or_red_setter(cross_or_red="red")
                    try:
                        red_detected = rospy.wait_for_message(f"/{self.vehicle_name}/red_cross_line_detect",Float64, timeout=1).data
                    except rospy.ROSException:
                        red_detected = False
                    if red_detected:
                        self.stop()
                        rospy.loginfo("Red cross line detected")
                        end = False
                        break


        self.LF.stop()
        #rospy.spin()

        # done = self.LF.lane_follow(10,rate)



