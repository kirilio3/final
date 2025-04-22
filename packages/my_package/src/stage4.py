#!/usr/bin/env python3

import rospy
import os
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage, Range
from std_msgs.msg import ColorRGBA, Float64, Float32
from turbojpeg import TurboJPEG
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
import math
from dt_apriltags import Detector
from cv_bridge import CvBridge

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = True
ENGLISH = False
SAFETY = False
AUSSIE = False

class LaneFollowNode():

    def __init__(self,ve, node_name, stall_number=1, ):
        # super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = ve
        self.stall_number = stall_number  # Stall number (1, 2, 3, or 4)
        self.state = 'LANE_FOLLOWING'  # States: LANE_FOLLOWING, TURNING_TO_STALL, PARKING

        # Map stall number to AprilTag ID and side
        self.stall_config = {
            1: {'id': 44, 'side': 'right'},
            2: {'id': 58, 'side': 'right'},
            3: {'id': 13, 'side': 'left'},
            4: {'id': 47, 'side': 'left'}
        }

        # New variables for LED control
        self.led_colors = {
            'red': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            # 'red': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            'green': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        }
        self.num_leds = 5  # Duckiebot typically has 5 LEDs

        if self.stall_number not in self.stall_config:
            rospy.logerr(f"Invalid stall number {self.stall_number}. Choose 1, 2, 3, or 4.")
            rospy.signal_shutdown("Invalid stall number")
        self.target_tag_id = self.stall_config[self.stall_number]['id']
        self.stall_side = self.stall_config[self.stall_number]['side']

        # Publishers & Subscribers
        if SAFETY:
            self.tof_sub = rospy.Subscriber(f"/{self.veh}/front_center_tof_driver_node/range",
                                            Range, self.cb_tof, queue_size=1)
        self.pub = rospy.Publisher(f"/{self.veh}/output/image/mask/compressed",
                                   CompressedImage, queue_size=1)
        # New LED publisher
        self.led_pub = rospy.Publisher(f"/{self.veh}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)

        self.sub = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed",
                                    CompressedImage, self.callback, queue_size=1, buff_size="20MB")
        self.vel_pub = rospy.Publisher(f"/{self.veh}/car_cmd_switch_node/cmd",
                                       Twist2DStamped, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(f"/{self.veh}/camera_node/camera_info",
                                                CameraInfo, self.cb_camera_info, queue_size=1)

        self.jpeg = TurboJPEG()
        self.bridge = CvBridge()
        self.loginfo("Initialized")

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # AprilTag Detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Red line detection parameters
        self.red_lower1 = np.array([0, 100, 100], np.uint8)
        self.red_upper1 = np.array([10, 255, 255], np.uint8)
        self.red_lower2 = np.array([160, 100, 100], np.uint8)
        self.red_upper2 = np.array([180, 255, 255], np.uint8)
        self.red_line_detected = False
        self.red_line_distance = None
        self.red_line_y = None
        self.red_line_count = 0

        # Lane Following PID
        self.proportional = None
        self.offset = -180 if ENGLISH else 230
        if AUSSIE:
            self.offset = 0
        # self.velocity = 0.3
        self.velocity = 0.2
        
        self.twist = Twist2DStamped(v=self.velocity, omega=0)
        # self.P = 0.025
        # self.I = 0.0005
        # self.D = 0.0001
        self.P = 0.045
        self.I = 0.001
        self.D = 0.0001
        if AUSSIE:
            self.P = 0.0005
            self.D = -0.025
            self.I = 0.5
        self.last_error = 0
        self.integral = 0
        self.last_time = rospy.get_time()
        self.tof_distance = 1.0
        self.obj_stop = False

        # Parking PID
        self.parking_P = 0.02
        self.parking_I = 0.0005
        self.parking_D = 0.0001
        self.parking_integral = 0
        self.parking_last_error = 0
        self.parking_last_time = rospy.get_time()
        self.target_distance = 0.1  # 5 cm
        self.parking_velocity = 0.17  # Slow velocity for parking

        # Control parameters for turns
        self.angular_vel = 2.4
        self.omega_speed = 2.5

        # Parking state variables
        self.tag_detected = False
        self.tag_distance = None
        self.tag_center_x = None
        self.turn_start_time = None

        

        rospy.Rate(0.20).sleep()

        rospy.on_shutdown(self.hook)

    def set_led_color(self, color_name):
        """Set all LEDs to the specified color."""
        if color_name not in self.led_colors:
            rospy.logwarn(f"Unknown color: {color_name}")
            return
        
        led_msg = LEDPattern()
        color = self.led_colors[color_name]
        led_msg.rgb_vals = [color] * self.num_leds
        self.led_pub.publish(led_msg)

    def cb_camera_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.D)

    def cb_tof(self, msg):
        self.tof_distance = msg.range
        if 0.05 < self.tof_distance <= 0.3:
            self.obj_stop = True

    def detect_apriltag(self, img):
        if self.camera_matrix is None or self.distortion_coeffs is None:
            self.logwarn("Camera calibration parameters not available")
            return None

        # Undistort and crop image
        undistorted = cv2.undistort(img, self.camera_matrix, self.distortion_coeffs)
        # undistorted = cv2.GaussianBlur(undistorted, (9, 9), 0)
        height, width = undistorted.shape[:2]
        crop_top = int(height // 4)
        crop_bottom = height
        crop_left = 0
        crop_right = width
        cropped = undistorted[crop_top:crop_bottom, crop_left:crop_right]

        # Convert to grayscale
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

        # Camera parameters for AprilTag detection
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        camera_params = (fx, fy, cx, cy)

        # Detect AprilTags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=0.065
        )

        # Filter for target tag ID
        for tag in tags:
            if tag.tag_id == self.target_tag_id:
                # Extract distance from pose (z-axis translation)
                distance = tag.pose_t[2][0]  # z-component in meters
                center_x = int(tag.center[0])
                if DEBUG:
                    # Draw bounding box and ID
                    output_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    for i in range(4):
                        pt1 = (int(tag.corners[i-1][0]), int(tag.corners[i-1][1]))
                        pt2 = (int(tag.corners[i][0]), int(tag.corners[i][1]))
                        cv2.line(output_img, pt1, pt2, (0, 255, 0), 2)
                    cv2.putText(output_img, str(tag.tag_id), (center_x, int(tag.center[1])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(output_img))
                    self.pub.publish(rect_img_msg)
                return {'distance': distance, 'center_x': center_x, 'width': gray.shape[1]}
        return None

    def callback(self, msg):
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        if self.state == 'LANE_FOLLOWING':
            # Yellow lane detection
            mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
            crop_masked = cv2.bitwise_and(crop, crop, mask=mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            max_area = 20
            max_idx = -1
            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    max_idx = i
                    max_area = area

            if max_idx != -1:
                M = cv2.moments(contours[max_idx])
                try:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    self.proportional = cx - int(crop_width / 2) + self.offset
                    if DEBUG:
                        cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                        cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
                except:
                    pass
            else:
                self.proportional = None

            # Red line detection
            red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.red_line_detected = False
            self.red_line_distance = None
            self.red_line_y = None
            for contour in red_contours:
                if cv2.contourArea(contour) > 30:
                # if cv2.contourArea(contour) > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cx = x + w // 2
                    cy = y + h // 2
                    self.red_line_y = cy
                    self.red_line_detected = True
                    if DEBUG:
                        cv2.circle(crop, (cx, cy), 5, (0, 0, 255), -1)
                    if self.camera_matrix is not None:
                        focal_length = 60
                        image_height = crop.shape[0]
                        pixel_y = cy
                        try:
                            distance = (focal_length * 0.1) / (image_height - pixel_y)
                            self.red_line_distance = max(0.01, min(distance, 1.0))
                        except ZeroDivisionError:
                            self.logwarn(f"ZeroDivisionError in red line distance calculation")
                            self.red_line_detected = False
                    else:
                        self.logwarn("Camera matrix not available")
                        self.red_line_detected = False
                    break

            if DEBUG:
                rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
                self.pub.publish(rect_img_msg)

        elif self.state in ['TURNING_TO_STALL', 'PARKING']:
            # Process image for AprilTag detection
            tag_info = self.detect_apriltag(img)
            if tag_info:
                self.tag_detected = True
                self.tag_distance = tag_info['distance']
                self.tag_center_x = tag_info['center_x']
                self.image_width = tag_info['width']
            else:
                self.tag_detected = False
                self.tag_distance = None
                self.tag_center_x = None

    def go_straight(self, time):
        duration = time
        rate = rospy.Rate(20)
        start_time = rospy.get_time()
        self.loginfo(f"Moving straight for {duration:.2f} seconds...")
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            cmd = Twist2DStamped(v=self.velocity * 1.1, omega=0.0)
            self.vel_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def turn_3(self, direction='right'):
        angle = math.pi / 2  # 90 degrees to face stalls
        omega = -self.angular_vel if direction == 'right' else self.angular_vel
        # duration = angle / abs(omega)
        duration = 1.35
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        self.loginfo(f"Turning {direction} for {duration:.2f} seconds...")
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            cmd = Twist2DStamped(v=0.65, omega=omega * (2 if direction == 'right' else 2))
            self.vel_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def turn_4(self, direction='right'):
        angle = math.pi / 1.1
        omega = -self.angular_vel if direction == 'right' else self.angular_vel
        duration = 2.6  # Tuned for stall 2 (left turn)
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        self.loginfo(f"Turning {direction} for {duration:.2f} seconds... {angle}")
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            cmd = Twist2DStamped(v=0.6, omega=omega * (2 if direction == 'right' else 1.1))
            self.vel_pub.publish(cmd)
            rate.sleep()
        self.stop()
    
    def turn_1(self, direction='right'):
        # angle = math.pi / 1.1  # 90 degrees to face stalls
        omega = -self.angular_vel if direction == 'right' else self.angular_vel
        # duration = angle / abs(omega)
        duration = 1.3
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        self.loginfo(f"Turning {direction} for {duration:.2f} seconds...")
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            cmd = Twist2DStamped(v=0.8, omega=omega)
            self.vel_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def turn(self, direction='right'):
        angle = math.pi / 2  # 90 degrees to face stalls
        omega = -self.angular_vel if direction == 'right' else self.angular_vel
        # duration = angle / abs(omega)
        duration = 0.5
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        self.loginfo(f"Turning {direction} for {duration:.2f} seconds...")
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            cmd = Twist2DStamped(v=0.7, omega=omega * (2 if direction == 'right' else 2))
            # cmd = Twist2DStamped(v=0.6, omega=omega)
            self.vel_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def park(self):
        if not self.tag_detected or self.tag_distance is None or self.tag_center_x is None:
            self.logwarn("Target AprilTag not detected")
            self.twist.v = 0
            self.twist.omega = 0
            self.vel_pub.publish(self.twist)
            return

        # Center the tag in the image (lateral alignment)
        error_x = self.tag_center_x - (self.image_width / 2)
        current_time = rospy.get_time()
        dt = current_time - self.parking_last_time
        self.parking_integral += error_x * dt
        d_error = (error_x - self.parking_last_error) / dt if dt > 0 else 0
        self.parking_last_error = error_x
        self.parking_last_time = current_time

        P = -error_x * self.parking_P
        I = -self.parking_integral * self.parking_I
        D = -d_error * self.parking_D
        omega = P + I + D

        # Distance control
        distance_error = self.tag_distance - self.target_distance
        velocity = min(self.parking_velocity, max(0.5, distance_error * 0.5))  # Slow down as we approach

        self.twist.v = velocity
        self.twist.omega = omega
        self.vel_pub.publish(self.twist)

        # Stop when within 5 cm ± 1 cm
        if abs(distance_error) < 0.01:
            self.loginfo("Reached target distance. Parking complete.")
            self.stop()
            rospy.signal_shutdown("Parking complete")

    def drive(self):
        if self.state == 'LANE_FOLLOWING':
            # Initialize lights to green
            # self.set_led_color('green')
            # Handle red line detection
            if self.red_line_detected and self.red_line_distance is not None:
                self.loginfo(f"Red line #{self.red_line_count + 1} detected at {self.red_line_distance:.2f} meters")
                if 0.05 <= self.red_line_distance <= 0.07:
                    self.stop(8)
                    rospy.sleep(1.0)
                    self.red_line_count += 1
                    if self.red_line_count == 1:
                        rospy.sleep(2.0)
                        self.loginfo("Fourth red line: Initiating parking...")
                        self.stop(8)
                        self.state = 'TURNING_TO_STALL'
                        self.turn_start_time = rospy.get_time()
                        
                        # Select turn method based on stall number
                        if self.stall_number == 1:
                            self.turn_1(direction=self.stall_side)
                        elif self.stall_number == 2:
                            self.go_straight(5)
                            self.turn(direction=self.stall_side)
                        elif self.stall_number == 3:
                            self.turn_3(direction=self.stall_side)
                        elif self.stall_number == 4:
                            self.turn_4(direction=self.stall_side)
                        
                        self.loginfo("Moving straight briefly after turn...")
                        self.go_straight(1)
                        self.state = 'PARKING'
                        return
                    self.red_line_detected = False
                    self.red_line_distance = None
                    return
                else:
                    self.loginfo(f"Red line detected but distance out of range")
                    self.red_line_detected = False
                    self.red_line_distance = None

            # Handle TOF stop
            if self.obj_stop:
                self.stop(8)
                self.obj_stop = False
                self.logwarn("Stopped due to TOF")
                rospy.sleep(1.0)
                return

            # Lane following
            if self.proportional is None:
                self.twist.omega = 0
                self.last_error = 0
            else:
                P = -self.proportional * self.P
                d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                self.last_error = self.proportional
                self.last_time = rospy.get_time()
                D = d_error * self.D
                current_time = rospy.get_time()
                dt = current_time - self.last_time
                self.integral += self.proportional * dt
                I = self.I * self.integral
                self.twist.v = self.velocity
                self.twist.omega = P + D + I

            self.vel_pub.publish(self.twist)

        elif self.state == 'PARKING':
            self.park()

    def stop(self, duration=8):
        self.twist.v = 0
        self.twist.omega = 0
        for _ in range(duration):
            self.vel_pub.publish(self.twist)

    def hook(self):
        self.loginfo("SHUTTING DOWN")
        self.stop()

    def part4run(self, n,name):
        # Example: Set stall_number via ROS parameter or command-line argument
        # stall_number = rospy.get_param('~stall_number', 2)  # Default to stall 1
        node = LaneFollowNode("lanefollow_node", stall_number=n,ve=name)
        rate = rospy.Rate(8)
        while not rospy.is_shutdown():
            node.drive()
            rate.sleep()