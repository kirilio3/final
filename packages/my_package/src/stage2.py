#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import ColorRGBA, Float64, Int64
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

class Stage2():

    def __init__(self, vehicle_name,ID1,ID2):

        self.vehicle_name = vehicle_name

        self.cross_walk_number =0
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.ID1 = ID1
        self.ID2 = ID2
        ##############following functiinalities are added#####################
        self.LF = lf.Lane_Following(self.vehicle_name)
        # self.DC = dc.Detect_Corss(self.vehicle_name, None,None, debugger=False)
        # self.DA = da.DetectApriltag(self.vehicle_name)

        self.distance = 10
        
        
        # Publishers
        self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
        self.red_cross_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/red_cross_line_detect", Float64, queue_size=1)
        self.line_type_pub = rospy.Publisher(f"/{self.vehicle_name}/line_type", Float64, queue_size=10)
        self.tag_id  = rospy.Publisher(f"/{self.vehicle_name}/tag_id", Int64, queue_size=10)
        
        # Subscribers

        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"

        # self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
        self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)
        self.red = 1
        self.cross = 0
    '''
        please set camera info here for every camera related node 
    '''
    def cb_camera_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.D)
        # self.DC.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
        self.LF.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
        # self.DA.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)


    def stop(self):
        self.LF.stop()
    

    def run_stage2(self):
        self.line_type_pub.publish(Float64(self.red))
        rospy.sleep(1)
        rate = 20

        while True:  
            while not rospy.is_shutdown(): 
                self.LF.lane_follow(10,rate)
                try:
                    red_detected = rospy.wait_for_message(f"/{self.vehicle_name}/red_cross_line_detect",Float64, timeout=1).data
                except rospy.ROSException:
                    red_detected = False
                print(red_detected)
                if red_detected:
                    rospy.loginfo("Red cross line detected")
                    rospy.sleep(1)
                    self.LF.stop()
                    break
            try: 
                tagID = rospy.wait_for_message(f"/{self.vehicle_name}/tag_id",Int64, timeout=1).data
            except:
                tagID = None
            print(tagID)
            rospy.loginfo(f"Tag ID: {tagID}")
            if tagID is not None:
                if tagID == self.ID1:
                    rospy.loginfo(f"Tag ID {tagID} detected")
                    self.LF.turn_left_90_degree_arc()
                elif tagID == self.ID2:
                    rospy.loginfo(f"Tag ID {tagID} detected")
                    self.LF.turn_right_90_degree_arc()
                    self.LF.stop()
                    rospy.sleep(2)
                    break

    def run_stage3(self):
        # self.DC.cross_or_red_setter("cross")
        self.line_type_pub.publish(Float64(self.cross))
        rospy.sleep(1)
        rate = 20
        end = True
        while end:  
            while not rospy.is_shutdown(): 
                self.LF.lane_follow(20,rate)
                if self.cross_walk_number < 2:
                    try:
                        cross_detected = rospy.wait_for_message(f"/{self.vehicle_name}/corss_line_detect",Float64, timeout=1).data
                    except rospy.ROSException:
                        cross_detected = False
                    if cross_detected:
                        self.cross_walk_number+=1
                        rospy.loginfo("cross line detected")
                        self.LF.stop()
                        rospy.sleep(1)
                        try:
                            ped_detected = rospy.wait_for_message(f"/{self.vehicle_name}/pedestrian_detect",Float64, timeout=5).data
                            rospy.loginfo(ped_detected)
                        except rospy.ROSException:
                            ped_detected = True
                        while ped_detected:
                            try:
                                ped_detected = rospy.wait_for_message(f"/{self.vehicle_name}/pedestrian_detect",Float64, timeout=5).data
                                print(ped_detected)
                            except rospy.ROSException:
                                ped_detected = True
                        print("im here")
                        self.LF.go_straight_for_half_meters()
                else:
                    # self.DC.cross_or_red_setter(cross_or_red="red")
                    self.line_type_pub.publish(Float64(self.red))
                    try:
                        red_detected = rospy.wait_for_message(f"/{self.vehicle_name}/red_cross_line_detect",Float64, timeout=1).data
                    except rospy.ROSException:
                        red_detected = False
                    if red_detected:
                        rospy.sleep(1)
                        self.stop()
                        rospy.loginfo("Red cross line detected")
                        end = False
                        break




