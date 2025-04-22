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



        self.led_colors = {
            'red': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            # 'red': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            'green': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        }
        self.num_leds = 5  # Duckiebot typically has 5 LEDs

        ##############following functiinalities are added#####################
        self.LF = lf.Lane_Following(self.vehicle_name)
        # self.DC = dc.Detect_Corss(self.vehicle_name, None,None, debugger=False)
        # self.DA = da.DetectApriltag(self.vehicle_name)

        self.distance = 10
        self.counter = 0
        
        # Publishers
        self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
        self.red_cross_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/red_cross_line_detect", Float64, queue_size=1)
        self.line_type_pub = rospy.Publisher(f"/{self.vehicle_name}/line_type", Float64, queue_size=10)
        self.tag_id= None
        self.apriltag_type_pub = rospy.Publisher(f"/{self.vehicle_name}/apriltag_type", Int64, queue_size=10)
        self.led_pub = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.send_of_sig_pub = rospy.Publisher(f"/{self.vehicle_name}/send_of_sig", Int64, queue_size=1)
        # self.sign_change_pub = rospy.Publisher(f"/{self.vehicle_name}/sign_change", Int64, queue_size=10)
        # Subscribers
        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        self.linetype = rospy.Subscriber(f"/{self.vehicle_name}/red_cross_line_detect"
            ,Float64,
            self.red_cross_line_cb,
            queue_size=10,)
        self.tag_id_sub = rospy.Subscriber(f"/{self.vehicle_name}/tag_id",Int64, self.tag_id_cb, queue_size=1)
        self.cross_line_detect = rospy.Subscriber(f"/{self.vehicle_name}/corss_line_detect",Float64, self.cross_line_detect_cb, queue_size=1)
        self.ped_detection = rospy.Subscriber(f"/{self.vehicle_name}/pedestrian_detect",Float64, self.ped_detection_cb, queue_size=1)
        self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)
        # self.duckiebot_detect_sub = rospy.Subscriber(f"/{self.vehicle_name}/duckiebot_detected", Int64, self.duckiebot_detected_cb, queue_size=1)
        self.duckiebot_distance_sub = rospy.Subscriber(f"/{self.vehicle_name}/duckiebot_distance", Float64, self.duckiebot_distance_cb, queue_size=1)
        self.cross_line_detect = 0
        # self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
        self.duckiebot_distance = -1
        self.duckiebot_detected = 0
        self.ped = 0
        self.red = 1
        self.cross = 0
        self.line_reached = 0
        

        self.set_led_color("green")
    '''
        please set camera info here for every camera related node 
    '''
    def cb_camera_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.D)
        # self.DC.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
        self.LF.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)
        # self.DA.camera_info_setter(camera_matrix=self.camera_matrix, distortion_coeffs=self.distortion_coeffs)

    def tag_id_cb(self, msg):
        self.tag_id = msg.data
    def duckiebot_detected_cb(self, msg):
        self.duckiebot_detected = msg.data
    def duckiebot_distance_cb(self, msg):
        self.duckiebot_distance = msg.data

    def red_cross_line_cb(self, msg):
        self.line_reached = msg.data
    def cross_line_detect_cb(self, msg):
        self.cross_line_detect = msg.data

    def ped_detection_cb(self, msg):
        self.ped = msg.data
    def stop(self):
        self.LF.stop()
    

    def run_stage1(self):
        # self.set_led_color("RED")
        stage = "bot"
        error = 0
        self.apriltag_type_pub.publish(Int64(1))
        while not rospy.is_shutdown():
            while self.duckiebot_distance !=-1 and self.duckiebot_distance <= 0.25:
                self.LF.stop(time=0.5)
                if self.duckiebot_distance>0.25:
                    break
            while self.duckiebot_distance != -1 and self.duckiebot_distance > 0.25 and self.duckiebot_distance<=0.5:
                if stage != "bot":
                    # self.set_led_color("red")
                    stage = "bot"
                error= self.LF.Bot_following(20,self.duckiebot_distance)
                # print(error)
                if self.line_reached :
                    print(self.counter)
                    self.counter+=1
                    rospy.loginfo("Red cross line detected")
                    # rospy.sleep(0.3)
                    self.LF.stop()
                    print(error)
                    if error < 320 and abs(320-error) >100:
                        self.LF.turn_left_90_degree_arc()
                    elif error > 320 and abs(320-error) >100:
                        self.LF.turn_right_90_degree_arc()

                    if self.counter >=3: 
                        break
                # print(self.duckiebot_distance)
                
            if self.duckiebot_distance == -1:
                if stage != "lane":
                    # self.set_led_color("green")
                    stage = "lane"
                self.LF.lane_follow(10, 20)
                if self.line_reached :
                    print(self.counter)
                    self.counter+=1
                    rospy.loginfo("Red cross line detected")
                    # rospy.sleep(0.3)
                    self.LF.stop()
                    print(error)
                    if error < 320 and abs(320-error) >100:
                        self.LF.turn_left_90_degree_arc()
                    elif error > 320 and abs(320-error) >100:
                        self.LF.turn_right_90_degree_arc()
                    if self.counter >=3: 
                        break


    def run_stage2(self):
        self.apriltag_type_pub.publish(Int64(0))
        self.line_type_pub.publish(Float64(self.red))
        rospy.sleep(1)
        rate = 10

        while True:  
            while not rospy.is_shutdown(): 
                self.LF.lane_follow(10,rate)
                tagID = self.tag_id
                if self.line_reached:
                    rospy.loginfo("Red cross line detected")
                if self.line_reached and tagID is not None:
                    rospy.loginfo("Red cross line detected")
                    self.LF.stop()
                    # self.LF.go_straight(0.5,0.25,time_to_stop=0)
                    # self.LF.turn_left(1.57079633,4)
                    # self.LF.go_straight(0.1,0.25,time_to_stop=0)

                    break

            rospy.loginfo(f"Tag ID: {tagID}")
            if tagID is not None:
                if tagID == self.ID1:
                    rospy.loginfo(f"Tag ID {tagID} detected")
                    self.LF.turn_left_90_degree_arc()
                    self.tag_id = None
                elif tagID == self.ID2:
                    rospy.loginfo(f"Tag ID {tagID} detected")
                    self.LF.stop(1)
                    # self.LF.go_straight(0.3,0.25,time_to_stop=0)
                    # self.LF.turn_right(1.57079633,4)
                    # self.LF.go_straight(0.1,0.25,time_to_stop=0)
                    self.LF.turn_right_90_degree_arc()
                    print("after")
                    self.LF.stop()
                    break

    def run_stage3(self):
        # self.DC.cross_or_red_setter("cross")
        self.apriltag_type_pub.publish(Int64(0))
        self.line_type_pub.publish(Float64(self.cross))
        rospy.sleep(1)
        rate = 10
        end = True
        while end:  
            while not rospy.is_shutdown(): 
                self.LF.lane_follow(20,rate)
                # print(self.duckiebot_distance)
                # if self.duckiebot_distance != -1 and self.duckiebot_distance <= 0.25:
                #     self.LF.stop()
                #     rospy.loginfo("Duckiebot detected")
                #     self.LF.turn_left(0.785398163, 4)
                #     self.LF.go_straight(0.38,0.25)
                #     self.LF.turn_right(1.04719755, 4)
                #     # counter = 0
                #     # flag = True
                #     # self.sign_change_pub.publish(Int64(-1))
                #     # self.duckiebot_distance_sub.unregister()
                #     # self.duckiebot_distance = None
                #     # self.LF.send_cmd(v=0.25, omega=5,time=0.5)
                #     # self.LF.go_straight_for_half_meters(time=0.5,speed_multiplier=1.5)
                #     # self.LF.stop()
                #     # self.LF.send_cmd(v=0.2, omega=-3.5,time=0.5)
                #     self.apriltag_type_pub.publish(Int64(0))
                #     self.duckiebot_distance = -1
                #     rospy.sleep(1)


                if self.cross_walk_number < 3:
                    cross_line_detected = self.cross_line_detect
                    # rospy.loginfo(f"cross line detected: {cross_line_detected}")
                    if cross_line_detected:
                        
                        if self.cross_walk_number !=1:
                            # rospy.loginfo("cross line detected")
                            self.LF.stop()
                            rospy.sleep(1)
                            ped_detected = self.ped
                            while ped_detected:
                                ped_detected = self.ped
                            # s
                            self.LF.go_straight(0.4,0.25,time_to_stop=0)
                        else:
                            self.LF.stop()
                            rospy.loginfo("Duckiebot detected")
                            self.LF.turn_left(0.785398163, 4)
                            self.LF.go_straight(0.38,0.25)
                            self.LF.turn_right(0.785398163, 4)
                        self.cross_walk_number+=1

                else:
                    # self.DC.cross_or_red_setter(cross_or_red="red")
                    self.line_type_pub.publish(Float64(self.red))
                    red_detected = self.line_reached
                    if red_detected:
                        rospy.sleep(0.5)
                        self.stop()
                        # rospy.loginfo("Red cross line detected")
                        end = False
                        break
        self.send_of_sig_pub.publish(Int64(1))


    # def set_led_color(self, color):
    #     pattern = LEDPattern()
    #     colors = {
    #         'GREEN': ColorRGBA(0, 1, 0, 1),
    #         'RED': ColorRGBA(1, 0, 0, 1),
    #         'CYAN': ColorRGBA(0, 1, 1, 1),
    #         'WHITE': ColorRGBA(1, 1, 1, 1),
    #         'BLUE': ColorRGBA(0, 0, 1, 1),
    #     }
    #     selected_color = colors.get(color, ColorRGBA(0.5, 0, 0.5, 1))
    #     pattern.color_list = [color] * 5
    #     pattern.rgb_vals = [selected_color] * 5
    #     pattern.color_mask = [1] * 5
    #     pattern.frequency = 1.0
    #     pattern.frequency_mask = [1] * 5
    #     print("published")
    #     self.led_pub.publish(pattern)

    def set_led_color(self, color_name):
        """Set all LEDs to the specified color."""
        if color_name not in self.led_colors:
            rospy.logwarn(f"Unknown color: {color_name}")
            return

        led_msg = LEDPattern()
        color = self.led_colors[color_name]
        led_msg.rgb_vals = [color] * self.num_leds
        self.led_pub.publish(led_msg)

