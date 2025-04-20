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
from dt_apriltags import Detector
from collections import deque
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from mother_of_all import MotherOfAll as MOA


class Lane_Following(MOA):
    
    def __init__(self, vehicle_name, distortion_coeffs=None, camera_matrix=None):
        self.vehicle_name = vehicle_name
        


        # Lane detection variables
        self.bridge = CvBridge()
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs

        # Encoder variables
        self.last_left_ticks = None
        self.last_right_ticks = None
        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        
        # Parameters
        self.TICKS_PER_REV = 135
        self.WHEEL_RADIUS = 0.0318
        self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
        self.BASELINE = 0.077
        self.VELOCITY = 0.25
        self.OMEGA_SPEED = 2.5
        self.angular_vel = 2.6
        self.counter =0
        # Control parameters
        self.KP = 0.1
        self.KI = 0.0
        self.KD = 0.005
        self.TARGET_DISTANCE = 20  # meters

        self.deriv = 0.0

        # Variables for PID terms
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.error_window = deque(maxlen=5)
        
        # Publishers
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.yellow_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/yellow_lane", Float64, queue_size=1)
        self.white_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/white_lane", Float64, queue_size=1)
        self.corss_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/corss_line_detect", Float64, queue_size=1)
        self.color = rospy.Publisher(f"/{self.vehicle_name}/color", Float64, queue_size=1)
        self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)

        # Subscribers
        self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"

        
        self.sub_left_enc = rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
        self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)
        self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)

        self.yellow_msg_sub = rospy.Subscriber(f"/{self.vehicle_name}/yellow_lane", Float64, self.yellow_msg_cb,queue_size=1,)
        self.yellow_msg = 0
        self.white_msg_sub = rospy.Subscriber(f"/{self.vehicle_name}/white_lane", Float64, self.white_msg_cb,queue_size=1,)
        self.white_msg =0

        self.yellow_lower = np.array([20, 100, 100], np.uint8)
        self.yellow_upper = np.array([30, 255, 255], np.uint8)
        self.white_lower = np.array([0, 0, 200], np.uint8)
        self.white_upper = np.array([180, 30, 255], np.uint8)

        self.cross_detected = 0

    def cb_camera(self, msg):
        if self.camera_matrix is None or self.distortion_coeffs is None:
            return
            
        # Process image for lane detection and visualization
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
        undistorted_image = cv2.GaussianBlur(undistorted_image, (5, 5), 0)
        
        # Crop the image (e.g., lower half of 640x480 image)
        height, width = undistorted_image.shape[:2]
        crop_top = int(height // 2)  # Start from halfway down (240 for 480 height)
        crop_bottom = height    # Go to the bottom (480)
        crop_left = 0           # Start from the left edge
        crop_right = width      # Go to the right edge (640)
        cropped_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
        # Detect lanes and mark centers on the cropped image
        yellow_pos, white_pos, processed_image = self.detect_lanes(cropped_image)
        
        # Adjust lane positions to account for cropping offset (only x-coordinate matters here)
        if yellow_pos is not None:
            yellow_pos += crop_left  # Adjust x-coordinate if crop_left != 0
        if white_pos is not None:
            white_pos += crop_left   # Adjust x-coordinate if crop_left != 0
        
        # # Publish the processed cropped image
        # undistorted_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
        # self.image_pub.publish(undistorted_msg)
        # undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
        # self.image_pub.publish(undistorted_msg)
        
        # Publish lane detection results
        if yellow_pos is not None:
            # self.yellow_lane_pub.publish(Float64(yellow_pos))
            self.yellow_msg = yellow_pos
        else:
            self.yellow_msg = None
        if white_pos is not None:
            self.white_lane_pub.publish(Float64(white_pos))
            self.white_msg = white_pos
        else:
            self.white_msg = None

    def yellow_msg_cb(self, msg):
        self.yellow_msg = msg.data
        # rospy.loginfo(f"Yellow lane position: {self.yellow_msg}")
        # self.yellow_lane_pub.publish(Float64(self.yellow_msg))
    def white_msg_cb(self,msg):
        self.white_msg = msg.data
        # rospy.loginfo(f"White lane position: {self.white_msg}")
        # self.white_lane_pub.publish(Float64(self.white_msg))

    def detect_lanes(self, image):
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
        yellow_center = None
        white_center = None

        # Detect yellow lane
        yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        centers = []
        for contour in yellow_contours:
            if cv2.contourArea(contour) > 300:  # Adjust threshold based on dot size
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
                    # Optionally draw each dot's center:
                    #cv2.circle(image, (cx, cy), 3, (0, 255, 255), -1)

        if centers:
            avg_cx = int(sum(pt[0] for pt in centers) / len(centers))
            avg_cy = int(sum(pt[1] for pt in centers) / len(centers))
            yellow_center = (avg_cx, avg_cy)
            # Draw the averaged center
            yellow_pos = avg_cx
            cv2.circle(image, yellow_center, 5, (0, 255, 255), -1)  # Yellow marker
        else:
            yellow_pos = None
            yellow_center = None

        # Now detect white lane and only consider white contours to the right of the yellow lane
        white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_pos = None
        white_y_value = None
        for contour in white_contours:
            if cv2.contourArea(contour) > 300:
                x, y, w, h = cv2.boundingRect(contour)
                center_white = x + w // 2
                # Only accept white contour if its center is to the right of the yellow lane's center
                if yellow_pos is None or center_white > yellow_pos:
                    white_pos = center_white
                    white_y_value = int(y + h // 2)
                    white_center = (center_white, white_y_value)
                    cv2.circle(image, white_center, 5, (255, 255, 255), -1)  # White marker
                    break  # Use the first valid white contour
                else:
                    white_pos = yellow_pos -1000   # Assume a fixed offset if white lane is to the left of yellow lane 

        if yellow_center is not None and white_pos is None:
            white_pos = yellow_pos -500
        # Calculate and draw the lane center if both lane centers are detected
        if yellow_center is not None and white_center is not None:
            lane_center_x = (yellow_center[0] + white_center[0]) // 2
            lane_center_y = (yellow_center[1] + white_center[1]) // 2
            lane_center = (lane_center_x, lane_center_y)
            cv2.circle(image, lane_center, 5, (0, 0, 255), -1)  # Red marker for lane center
            cv2.putText(image, f"Line Center: {lane_center}", (lane_center_x + 10, lane_center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Draw the screen center
        h, w = image.shape[:2]
        screen_center = (w // 2, h // 2)
        cv2.circle(image, screen_center, 5, (255, 0, 0), -1)  # Blue marker for screen center
        cv2.putText(image, f"Screen Center: {screen_center}", (screen_center[0] + 10, screen_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return yellow_pos, white_pos, image

    def camera_info_setter(self, camera_matrix, distortion_coeffs):
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs

    def cb_left_encoder(self, msg):
        current_ticks = msg.data
        if self.last_left_ticks is None:
            self.last_left_ticks = current_ticks
            return
        delta_ticks = current_ticks - self.last_left_ticks
        if delta_ticks > self.TICKS_PER_REV / 2:
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -self.TICKS_PER_REV / 2:
            delta_ticks += self.TICKS_PER_REV
        self.last_left_ticks = current_ticks
        distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
        self._left_distance_traveled += distance

    def cb_right_encoder(self, msg):
        current_ticks = msg.data
        if self.last_right_ticks is None:
            self.last_right_ticks = current_ticks
            return
        delta_ticks = current_ticks - self.last_right_ticks
        if delta_ticks > self.TICKS_PER_REV / 2:
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -self.TICKS_PER_REV / 2:
            delta_ticks += self.TICKS_PER_REV
        self.last_right_ticks = current_ticks
        distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
        self._right_distance_traveled += distance
    
    def stop(self,time = 2):
        for i in range(5):
            rospy.loginfo("Stopping vehicle...")
            msg = Twist2DStamped(v=0.0, omega=0.0)
            self.pub_cmd.publish(msg)
        rospy.sleep(time)


    def pid_control(self, error):
        now = rospy.get_time()
        dt  = max(now - self.prev_time, 1e-3)
        self.error_window.append(error)
        smooth_error = sum(self.error_window) / len(self.error_window)
        # P
        P = self.KP * smooth_error

        # if self.counter == 0:
        #     self.prev_integral = error * dt
        # if self.counter == 20:
        #     self.counter = 0
        #     self.integral -=self.prev_integral
        # self.counter+=1
        
        # I with anti‐windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)
        I = self.KI * self.integral

        # D with low‐pass filter
        rawD = (smooth_error - self.prev_error) / dt
        self.deriv = 0.8*self.deriv + 0.2*rawD
        D = self.KD * self.deriv

        omega = P + I + D
        omega = max(min(omega, self.OMEGA_SPEED), -self.OMEGA_SPEED)

        cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
        self.pub_cmd.publish(cmd)

        self.prev_error = smooth_error
        self.prev_time  = now

    def lane_follow(self, distance,rate=30):
        # while (True):
        #     pass
        # rospy.loginfo("Starting lane following for {distance} meters with PID control...")
        
        rate = rospy.Rate(rate)  # 20 Hz
        self.prev_time = rospy.get_time()
        
        # while not rospy.is_shutdown():
        avg_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2


        if avg_distance >= distance:
            self.stop()
            rospy.loginfo("Target distance reached!")
            return True
        
        if self.yellow_msg is not None and self.white_msg is not None:
            yellow_msg = self.yellow_msg
            white_msg = self.white_msg
            # print(yellow_msg, white_msg)
        else:
            yellow_msg = None
            white_msg = None

        
        # pixel_to_meter = 0.008  # e.g. measured by experiment
        if yellow_msg is not None and white_msg is not None:
            # print(yellow_msg, white_msg)
            lane_center = (yellow_msg+ white_msg) / 2
            image_center = 320  # Assuming 640x480 image
            # print(lane_center)
            real_error = lane_center - image_center
            self.pid_control(real_error)
        else:
            # If lanes not detected, go straight slowly
            cmd = Twist2DStamped(v=self.VELOCITY*1.5 , omega=self.OMEGA_SPEED*1.5)
            self.pub_cmd.publish(cmd)
            # rospy.logwarn("Lanes not detected, moving straight slowly")
            
        rate.sleep()
        # except self.getStopException as e:
        #     # rospy.loginfo(e)
        #     self.stop()
            
        #     while True:
        #         try:
        #             pedestrian_detected = rospy.wait_for_message(f"/{self.vehicle_name}/pedestrian_detect",Float64,timeout=5.0)
        #             if not pedestrian_detected.data:
        #                 rospy.loginfo(" no1")
        #                 break
        #             rospy.loginfo("duck duck")
        #         except:
        #             rospy.loginfo(" waiting ")
        #             # break
        #     cmd = Twist2DStamped(v=self.VELOCITY , omega=0)
        #     self.pub_cmd.publish(cmd)
        #     rospy.sleep(2)


    def turn_right_90_degree_arc(self):
        rospy.loginfo("Turning right 90 degrees...")
        cmd = Twist2DStamped(v=self.VELOCITY*2, omega=-self.angular_vel*2)
        # for i in range(3):
        self.pub_cmd.publish(cmd)
        rospy.sleep(1)
    def turn_left_90_degree_arc(self):
        rospy.loginfo("Turning left 90 degrees...")
        cmd = Twist2DStamped(v=self.VELOCITY*2, omega=self.angular_vel*0.75)
        # for i in range(3):
        self.pub_cmd.publish(cmd)
        rospy.sleep(2)
    def go_straight_for_half_meters(self, time=1.5,speed_multiplier=2):
        rospy.loginfo("Going straight for 0.5 meters...")
        cmd = Twist2DStamped(v=self.VELOCITY*speed_multiplier, omega=0)
        # for i in range(3):
        self.pub_cmd.publish(cmd)
        rospy.sleep(1.5)
    def send_cmd(self, v, omega, time=0):
        msg = Twist2DStamped(v=v, omega=omega)
        self.pub_cmd.publish(msg)
        rospy.sleep(time)
    # def stop_with_cond(self, condition:bool, time):
    #     if condition:
    #         self.stop()
    #         rospy.loginfo(f"stop for {time} sec")
    #         rospy.sleep(time)
            