

############## lane following with four red stops (pretty good) (no duckiebot following yet)

# #!/usr/bin/env python3

# import os
# import math
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from std_msgs.msg import ColorRGBA, Float64
# from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
# from sensor_msgs.msg import CompressedImage, CameraInfo
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import signal
# import sys

# class LaneFollowing(DTROS):
    
#     def __init__(self, node_name="lane_following_node"):
#         # Initialize the DTROS node
#         super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        
#         # Lane detection variables
#         self.bridge = CvBridge()
#         self.camera_matrix = None
#         self.distortion_coeffs = None

#         # Encoder variables
#         self.last_left_ticks = None
#         self.last_right_ticks = None
#         self._left_distance_traveled = 0.0
#         self._right_distance_traveled = 0.0
        
#         # Add red color ranges for detection
#         self.red_lower1 = np.array([0, 100, 100], np.uint8)
#         self.red_upper1 = np.array([10, 255, 255], np.uint8)
#         self.red_lower2 = np.array([170, 100, 100], np.uint8)
#         self.red_upper2 = np.array([180, 255, 255], np.uint8)

#         # Red line distance estimation
#         self.RED_LINE_Y_THRESHOLD = 200
#         self.RED_LINE_DISTANCE = 0.03
#         self.CAMERA_HEIGHT = 0.10
#         self.FOCAL_LENGTH = 50.0

#         # Parameters
#         self.TICKS_PER_REV = 135
#         self.WHEEL_RADIUS = 0.0318
#         self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
#         self.BASELINE = 0.077
#         self.VELOCITY = 0.2
#         self.OMEGA_SPEED = 2.5
#         self.angular_vel = 3

#         # Control parameters
#         self.KP = 0.015
#         self.KI = 0.0001
#         self.KD = 0.01
#         self.TARGET_DISTANCE = 20
        
#         # Variables for PID terms
#         self.prev_error = 0.0
#         self.integral = 0.0
#         self.prev_time = None

#         self.yellow_lower = np.array([20, 100, 100], np.uint8)
#         self.yellow_upper = np.array([30, 255, 255], np.uint8)
#         self.white_lower = np.array([0, 0, 200], np.uint8)
#         self.white_upper = np.array([180, 30, 255], np.uint8)

#         self.cross_detected = 0
        
#         # New variables for red line counting and state
#         self.red_line_count = 0  # Tracks number of red lines encountered
#         self.state = 'LANE_FOLLOWING'  # States: LANE_FOLLOWING, STOPPED, TURNING, STRAIGHT, FINISHED
#         self.last_red_detected = False  # To detect rising edge of red line

#         # Publishers
#         twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
#         self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
#         self.yellow_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/yellow_lane", Float64, queue_size=1)
#         self.white_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/white_lane", Float64, queue_size=1)
#         self.corss_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/corss_line_detect", Float64, queue_size=1)
#         self.color = rospy.Publisher(f"/{self.vehicle_name}/color", Float64, queue_size=1)
#         self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
#         self.red_line_pub = rospy.Publisher(f"/{self.vehicle_name}/red_line", Float64, queue_size=1)

#         # Subscribers
#         self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
#         self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
#         self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
#         self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        
#         self.sub_left_enc = rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
#         self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)
#         self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
#         self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)

#         # Handle Ctrl+C gracefully
#         signal.signal(signal.SIGINT, self.signal_handler)

#     def cb_camera_info(self, msg):
#         self.camera_matrix = np.array(msg.K).reshape(3, 3)
#         self.distortion_coeffs = np.array(msg.D)

#     def cb_camera(self, msg):
#         if self.camera_matrix is None or self.distortion_coeffs is None:
#             rospy.logwarn("Camera parameters not yet received, skipping image processing.")
#             return
            
#         image = self.bridge.compressed_imgmsg_to_cv2(msg)
#         undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
#         undistorted_image = cv2.GaussianBlur(undistorted_image, (5, 5), 0)
        
#         height, width = undistorted_image.shape[:2]
#         crop_top = height // 2
#         crop_bottom = height
#         crop_left = 0
#         crop_right = width
#         cropped_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
        
#         yellow_pos, white_pos, processed_image = self.detect_lanes(cropped_image)
        
#         if yellow_pos is not None:
#             yellow_pos += crop_left
#         if white_pos is not None:
#             white_pos += crop_left
        
#         undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
#         # self.image_pub.publish(undistorted_msg)
        
#         if yellow_pos is not None:
#             self.yellow_lane_pub.publish(Float64(yellow_pos))
#         if white_pos is not None:
#             self.white_lane_pub.publish(Float64(white_pos))

#     def detect_lanes(self, image):
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
#         yellow_center = None
#         white_center = None
#         red_detected = 0.0

#         # Yellow lane detection
#         yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
#         yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         centers = []
#         for contour in yellow_contours:
#             if cv2.contourArea(contour) > 300:
#                 M = cv2.moments(contour)
#                 if M["m00"] != 0:
#                     cx = int(M["m10"] / M["m00"])
#                     cy = int(M["m01"] / M["m00"])
#                     centers.append((cx, cy))
#         if centers:
#             avg_cx = int(sum(pt[0] for pt in centers) / len(centers))
#             avg_cy = int(sum(pt[1] for pt in centers) / len(centers))
#             yellow_center = (avg_cx, avg_cy)
#             yellow_pos = avg_cx
#             cv2.circle(image, yellow_center, 5, (0, 255, 255), -1)
#         else:
#             yellow_pos = None

#         # White lane detection
#         white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
#         white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         white_pos = None
#         for contour in white_contours:
#             if cv2.contourArea(contour) > 300:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 center_white = x + w // 2
#                 if yellow_pos is None or center_white > yellow_pos:
#                     white_pos = center_white
#                     white_y_value = int(y + h // 2)
#                     white_center = (center_white, white_y_value)
#                     cv2.circle(image, white_center, 5, (255, 255, 255), -1)
#                     break
#                 else:
#                     white_pos = yellow_pos + 500
#         if yellow_center is not None and white_pos is None:
#             white_pos = yellow_pos + 500

#         # Red line detection
#         red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
#         red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
#         red_mask = cv2.bitwise_or(red_mask1, red_mask2)
#         red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
#         for contour in red_contours:
#             if cv2.contourArea(contour) > 500:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 y_bottom = y + h
#                 image_height = image.shape[0]
#                 principal_point_y = image_height / 2
#                 pixel_offset_y = y_bottom - principal_point_y
#                 if pixel_offset_y > 0:
#                     distance = (self.FOCAL_LENGTH * self.CAMERA_HEIGHT) / pixel_offset_y
#                     if 0.025 <= distance <= 0.045:
#                         red_detected = 1.0
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
#                         cv2.putText(image, f"Red Line: {distance:.2f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#                     else:
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
#                         cv2.putText(image, f"Red Line: {distance:.2f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
#                 break

#         # Draw lane center
#         if yellow_center is not None and white_center is not None:
#             lane_center_x = (yellow_center[0] + white_center[0]) // 2
#             lane_center_y = (yellow_center[1] + white_center[1]) // 2
#             lane_center = (lane_center_x, lane_center_y)
#             cv2.circle(image, lane_center, 5, (0, 0, 255), -1)
#             cv2.putText(image, f"Line Center: {lane_center}", (lane_center_x + 10, lane_center_y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         # Draw screen center
#         h, w = image.shape[:2]
#         screen_center = (w // 2, h // 2)
#         cv2.circle(image, screen_center, 5, (255, 0, 0), -1)
#         cv2.putText(image, f"Screen Center: {screen_center}", (screen_center[0] + 10, screen_center[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#         self.red_line_pub.publish(Float64(red_detected))
#         return yellow_pos, white_pos, image

#     def cb_left_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_left_ticks is None:
#             self.last_left_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_left_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_left_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._left_distance_traveled += distance

#     def cb_right_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_right_ticks is None:
#             self.last_right_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_right_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_right_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._right_distance_traveled += distance

#     def pid_control(self, error):
#         current_time = rospy.get_time()
#         dt = current_time - self.prev_time if self.prev_time is not None and current_time > self.prev_time else 0.001
#         self.integral += error * dt
#         error_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
#         omega = (self.KP * error) + (self.KI * self.integral) + (self.KD * error_derivative)
#         omega = max(min(omega, self.OMEGA_SPEED), -self.OMEGA_SPEED)
#         # cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
        
#         if error > 15:
#             cmd = Twist2DStamped(v=self.VELOCITY/1.5, omega=omega*2)
#         else:
#             cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)

#         rospy.loginfo(f"Error: {error}")
#         rospy.loginfo(f"Derivative: {error_derivative}")

#         self.pub_cmd.publish(cmd)
#         self.prev_error = error
#         self.prev_time = current_time

#     def turn(self, direction='right'):
#         """Execute a 90-degree turn in the specified direction."""
#         # angle = math.pi / 2  # 90 degrees in radians
#         angle = math.pi / 1.5  # 90 degrees in radians

#         # rospy.loginfo(f"Angle: {angle}")
#         # rospy.loginfo(f"Angle 2: {math.pi / 3}")
#         # rospy.loginfo(f"Angle 3: {math.pi / 1.5}")
#         omega = -self.angular_vel if direction == 'right' else self.angular_vel
#         # duration = angle / abs(omega) + 0.5  # Time to turn 90 degrees
#         if direction == 'left':
#             duration = angle / abs(omega) + 1.3  # Time to turn 90 degrees
#         else:
#             duration = angle / abs(omega) + 0.5  # Time to turn 90 degrees
#         rate = rospy.Rate(10)
#         start_time = rospy.get_time()
        
#         rospy.loginfo(f"Turning {direction} for {duration:.2f} seconds...")
#         while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
#             if direction == 'left':
#                 cmd = Twist2DStamped(v=0.5, omega=omega*0.8)
#             else:
#                 cmd = Twist2DStamped(v=0.5, omega=omega*2)
#             self.pub_cmd.publish(cmd)
#             rate.sleep()
        
#         self.stop()
#         rospy.loginfo(f"Completed {direction} turn.")

#     def go_straight(self):
#         """Move straight slowly through the intersection."""
#         duration = 10.0  # Seconds to move straight (adjust as needed)
#         rate = rospy.Rate(20)
#         start_time = rospy.get_time()
        
#         rospy.loginfo(f"Moving straight for {duration:.2f} seconds...")
#         while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
#             cmd = Twist2DStamped(v=self.VELOCITY / 1.3, omega=0.0)
#             self.pub_cmd.publish(cmd)
#             rate.sleep()
        
#         self.stop()
#         rospy.loginfo("Completed straight movement.")

#     def lane_follow(self, rate=20):
#         """Modified lane following with red line handling."""
#         rospy.loginfo("Starting lane following with red line navigation...")
#         rate = rospy.Rate(rate)
#         self.prev_time = rospy.get_time()
        
#         while not rospy.is_shutdown():
#             if self.state == 'FINISHED':
#                 rospy.loginfo("Final red line reached, stopping.")
#                 self.stop()
#                 return True

#             try:
#                 yellow_msg = rospy.wait_for_message(f"/{self.vehicle_name}/yellow_lane", Float64, timeout=0.5)
#                 white_msg = rospy.wait_for_message(f"/{self.vehicle_name}/white_lane", Float64, timeout=0.5)
#                 red_msg = rospy.wait_for_message(f"/{self.vehicle_name}/red_line", Float64, timeout=0.5)
#             except rospy.ROSException:
#                 yellow_msg = None
#                 white_msg = None
#                 red_msg = None
                
#             # Handle red line detection (rising edge)
#             red_detected = red_msg is not None and red_msg.data == 1.0
#             if red_detected and not self.last_red_detected and self.state == 'LANE_FOLLOWING':
#                 self.red_line_count += 1
#                 self.state = 'STOPPED'
#                 self.stop()
#                 rospy.loginfo(f"Red line {self.red_line_count} detected, stopped.")
                
#                 # Execute action based on red line count
#                 if self.red_line_count == 1:
#                     rospy.sleep(2.0)  # Brief pause
#                     self.state = 'TURNING'
#                     # self.turn('left')
#                     self.turn('right')
#                     self.state = 'LANE_FOLLOWING'
#                 elif self.red_line_count == 2:
#                     rospy.sleep(2.0)
#                     self.state = 'STRAIGHT'
#                     self.go_straight()
#                     self.state = 'LANE_FOLLOWING'
#                 elif self.red_line_count == 3:
#                     rospy.sleep(2.0)
#                     self.state = 'TURNING'
#                     self.turn('left')
#                     self.state = 'LANE_FOLLOWING'
#                 elif self.red_line_count == 4:
#                     self.state = 'FINISHED'
#                 else:
#                     rospy.logwarn("Unexpected red line count, stopping.")
#                     self.state = 'FINISHED'
            
#             self.last_red_detected = red_detected

#             # Lane following logic
#             if self.state == 'LANE_FOLLOWING':
#                 if yellow_msg is not None and white_msg is not None:
#                     lane_center = (yellow_msg.data + white_msg.data) / 2
#                     image_center = 320  # Assuming 640x480 image
#                     error = image_center - lane_center
#                     self.pid_control(error)
#                 else:
#                     # No lanes detected (likely in intersection), move straight slowly
#                     cmd = Twist2DStamped(v=self.VELOCITY, omega=0.0)
#                     self.pub_cmd.publish(cmd)
#                     rospy.loginfo_throttle(2, "Lanes not detected, moving straight slowly.")
            
#             rate.sleep()

#     def stop(self):
#         msg = Twist2DStamped(v=0.0, omega=0.0)
#         self.pub_cmd.publish(msg)

#     def run(self):
#         """Main execution method."""
#         rospy.sleep(1)
#         self.lane_follow(rate=20)

#     def on_shutdown(self):
#         """Shutdown handler."""
#         self.stop()
#         rospy.loginfo("Lane following node shutting down.")
#         super(LaneFollowing, self).on_shutdown()

#     def signal_handler(self, sig, frame):
#         """Handle Ctrl+C."""
#         rospy.loginfo("Ctrl+C detected, shutting down...")
#         self.on_shutdown()
#         sys.exit(0)

# if __name__ == '__main__':
#     try:
#         node = LaneFollowing(node_name="lane_following_node")
#         node.run()
#     except rospy.ROSInterruptException:
#         pass

############## failed attempt at implementing 4 stop lines and interesection logic

# #!/usr/bin/env python3

# import os
# import math
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from std_msgs.msg import ColorRGBA, Float64
# from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
# from sensor_msgs.msg import CompressedImage, CameraInfo
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import signal
# import sys

# class LaneFollowing(DTROS):
    
#     def __init__(self, node_name="lane_following_node"):
#         super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        
#         self.bridge = CvBridge()
#         self.camera_matrix = None
#         self.distortion_coeffs = None

#         self.last_left_ticks = None
#         self.last_right_ticks = None
#         self._left_distance_traveled = 0.0
#         self._right_distance_traveled = 0.0
        
#         self.red_lower1 = np.array([0, 100, 100], np.uint8)
#         self.red_upper1 = np.array([10, 255, 255], np.uint8)
#         self.red_lower2 = np.array([170, 100, 100], np.uint8)
#         self.red_upper2 = np.array([180, 255, 255], np.uint8)

#         self.RED_LINE_Y_THRESHOLD = 200
#         self.RED_LINE_DISTANCE = 0.03  # 3 cm
#         self.CAMERA_HEIGHT = 0.10
#         self.FOCAL_LENGTH = 50.0  # Calibrate with camera_matrix

#         self.red_line_count = 0
#         self.last_red_line_distance = 0.0
#         self.MIN_DISTANCE_BETWEEN_RED = 0.2

#         self.TURN_OMEGA = 2.0
#         self.TURN_DURATION = 0.8  # ~90-degree turn (calibrate)
#         self.FORWARD_DISTANCE = 0.35  # Increased to clear ~30 cm intersection

#         self.TICKS_PER_REV = 135
#         self.WHEEL_RADIUS = 0.0318
#         self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
#         self.BASELINE = 0.077
#         self.VELOCITY = 0.2
#         self.OMEGA_SPEED = 2.5
#         self.angular_vel = 2.6
        
#         self.KP = 0.015
#         self.KI = 0.0001
#         self.KD = 0.01
#         self.TARGET_DISTANCE = 20
        
#         self.prev_error = 0.0
#         self.integral = 0.0
#         self.prev_time = None

#         self.yellow_lower = np.array([20, 100, 100], np.uint8)
#         self.yellow_upper = np.array([30, 255, 255], np.uint8)
#         self.white_lower = np.array([0, 0, 200], np.uint8)
#         self.white_upper = np.array([180, 30, 255], np.uint8)

#         self.cross_detected = 0
        
#         twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
#         self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
#         self.yellow_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/yellow_lane", Float64, queue_size=1)
#         self.white_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/white_lane", Float64, queue_size=1)
#         self.corss_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/corss_line_detect", Float64, queue_size=1)
#         self.color = rospy.Publisher(f"/{self.vehicle_name}/color", Float64, queue_size=1)
#         self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)
#         self.red_line_pub = rospy.Publisher(f"/{self.vehicle_name}/red_line", Float64, queue_size=1)

#         self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
#         self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
#         self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
#         self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        
#         self.sub_left_enc = rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
#         self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)
#         self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
#         self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)

#         signal.signal(signal.SIGINT, self.signal_handler)

#     def cb_camera_info(self, msg):
#         self.camera_matrix = np.array(msg.K).reshape(3, 3)
#         self.distortion_coeffs = np.array(msg.D)

#     def cb_camera(self, msg):
#         if self.camera_matrix is None or self.distortion_coeffs is None:
#             rospy.logwarn("Camera parameters not yet received, skipping image processing.")
#             return
            
#         image = self.bridge.compressed_imgmsg_to_cv2(msg)
#         undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
#         undistorted_image = cv2.GaussianBlur(undistorted_image, (5, 5), 0)
        
#         height, width = undistorted_image.shape[:2]
#         crop_top = height // 2
#         crop_bottom = height
#         crop_left = 0
#         crop_right = width
#         cropped_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
        
#         yellow_pos, white_pos, processed_image = self.detect_lanes(cropped_image)
        
#         if yellow_pos is not None:
#             yellow_pos += crop_left
#         if white_pos is not None:
#             white_pos += crop_left
        
#         undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
#         # self.image_pub.publish(undistorted_msg)
        
#         if yellow_pos is not None:
#             self.yellow_lane_pub.publish(Float64(yellow_pos))
#         if white_pos is not None:
#             self.white_lane_pub.publish(Float64(white_pos))

#     def detect_lanes(self, image):
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
#         yellow_center = None
#         white_center = None
#         red_detected = 0.0

#         yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
#         yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         centers = []
#         for contour in yellow_contours:
#             if cv2.contourArea(contour) > 300:
#                 M = cv2.moments(contour)
#                 if M["m00"] != 0:
#                     cx = int(M["m10"] / M["m00"])
#                     cy = int(M["m01"] / M["m00"])
#                     centers.append((cx, cy))

#         if centers:
#             avg_cx = int(sum(pt[0] for pt in centers) / len(centers))
#             avg_cy = int(sum(pt[1] for pt in centers) / len(centers))
#             yellow_center = (avg_cx, avg_cy)
#             yellow_pos = avg_cx
#             cv2.circle(image, yellow_center, 5, (0, 255, 255), -1)
#         else:
#             yellow_pos = None
#             yellow_center = None

#         white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
#         white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         white_pos = None
#         white_y_value = None
#         for contour in white_contours:
#             if cv2.contourArea(contour) > 300:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 center_white = x + w // 2
#                 if yellow_pos is None or center_white > yellow_pos:
#                     white_pos = center_white
#                     white_y_value = int(y + h // 2)
#                     white_center = (center_white, white_y_value)
#                     cv2.circle(image, white_center, 5, (255, 255, 255), -1)
#                     break
#                 else:
#                     white_pos = yellow_pos + 500

#         if yellow_center is not None and white_pos is None:
#             white_pos = yellow_pos + 500

#         red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
#         red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
#         red_mask = cv2.bitwise_or(red_mask1, red_mask2)
#         red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
#         for contour in red_contours:
#             if cv2.contourArea(contour) > 500:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 y_bottom = y + h
#                 image_height = image.shape[0]
#                 principal_point_y = image_height / 2
#                 pixel_offset_y = y_bottom - principal_point_y
#                 if pixel_offset_y > 0:
#                     distance = (self.FOCAL_LENGTH * self.CAMERA_HEIGHT) / pixel_offset_y
#                     rospy.loginfo(f"Distance to red line: {distance:.3f}m")
#                     if 0.03 <= distance <= 0.05:  # 2.5–3.5 cm
#                         red_detected = 1.0
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
#                         cv2.putText(image, f"Red Line: {distance:.3f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#                     else:
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
#                         cv2.putText(image, f"Red Line: {distance:.3f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
#                 break

#         self.red_line_pub.publish(Float64(red_detected))
#         return yellow_pos, white_pos, image

#     def cb_left_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_left_ticks is None:
#             self.last_left_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_left_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_left_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._left_distance_traveled += distance

#     def cb_right_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_right_ticks is None:
#             self.last_right_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_right_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_right_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._right_distance_traveled += distance
    
#     def stop(self):
#         msg = Twist2DStamped(v=0.0, omega=0.0)
#         self.pub_cmd.publish(msg)

#     def turn_right(self):
#         rospy.loginfo("Executing right turn")
#         cmd = Twist2DStamped(v=0.0, omega=self.TURN_OMEGA)
#         start_time = rospy.get_time()
#         rate = rospy.Rate(20)
#         while rospy.get_time() - start_time < self.TURN_DURATION and not rospy.is_shutdown():
#             self.pub_cmd.publish(cmd)
#             rate.sleep()
#         self.stop()

#     def turn_left(self):
#         rospy.loginfo("Executing left turn")
#         cmd = Twist2DStamped(v=0.0, omega=-self.TURN_OMEGA)
#         start_time = rospy.get_time()
#         rate = rospy.Rate(20)
#         while rospy.get_time() - start_time < self.TURN_DURATION and not rospy.is_shutdown():
#             self.pub_cmd.publish(cmd)
#             rate.sleep()
#         self.stop()

#     def move_forward(self, distance):
#         rospy.loginfo(f"Moving forward {distance} meters")
#         start_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#         cmd = Twist2DStamped(v=self.VELOCITY, omega=0.0)
#         rate = rospy.Rate(20)
#         while not rospy.is_shutdown():
#             current_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#             if current_distance - start_distance >= distance:
#                 self.stop()
#                 return
#             self.pub_cmd.publish(cmd)
#             rate.sleep()

#     def wait_for_lanes(self, timeout=5.0):
#         """Wait until yellow and white lanes are detected or timeout."""
#         rospy.loginfo("Waiting for lane detection after intersection")
#         start_time = rospy.get_time()
#         rate = rospy.Rate(20)
#         cmd = Twist2DStamped(v=self.VELOCITY / 2, omega=0.0)  # Move slowly straight
        
#         while not rospy.is_shutdown() and (rospy.get_time() - start_time) < timeout:
#             try:
#                 yellow_msg = rospy.wait_for_message(f"/{self.vehicle_name}/yellow_lane", Float64, timeout=1.0)
#                 white_msg = rospy.wait_for_message(f"/{self.vehicle_name}/white_lane", Float64, timeout=1.0)
#                 if yellow_msg is not None and white_msg is not None:
#                     rospy.loginfo("Lanes detected, resuming PID control")
#                     return True
#             except rospy.ROSException:
#                 pass
#             self.pub_cmd.publish(cmd)  # Keep moving slowly
#             rate.sleep()
        
#         rospy.logwarn("Timeout waiting for lanes, stopping")
#         self.stop()
#         return False

#     def pid_control(self, error):
#         current_time = rospy.get_time()
#         dt = current_time - self.prev_time if self.prev_time is not None and current_time > self.prev_time else 0.001
#         self.integral += error * dt
#         error_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
#         omega = (self.KP * error) + (self.KI * self.integral) + (self.KD * error_derivative)
#         omega = max(min(omega, self.OMEGA_SPEED), -self.OMEGA_SPEED)
#         cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
#         self.pub_cmd.publish(cmd)
#         self.prev_error = error
#         self.prev_time = current_time

#     def lane_follow(self, distance, rate=20):
#         rospy.loginfo(f"Starting lane following for {distance} meters with PID control...")
#         rate = rospy.Rate(rate)
#         self.prev_time = rospy.get_time()
        
#         while not rospy.is_shutdown():
#             avg_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#             if avg_distance >= distance:
#                 self.stop()
#                 rospy.loginfo("Target distance reached!")
#                 return True

#             try:
#                 yellow_msg = rospy.wait_for_message(f"/{self.vehicle_name}/yellow_lane", Float64, timeout=1.0)
#                 white_msg = rospy.wait_for_message(f"/{self.vehicle_name}/white_lane", Float64, timeout=1.0)
#                 red_msg = rospy.wait_for_message(f"/{self.vehicle_name}/red_line", Float64, timeout=1.0)
#             except rospy.ROSException as e:
#                 rospy.logwarn(f"Failed to get lane messages: {e}")
#                 yellow_msg = None
#                 white_msg = None
#                 red_msg = None
                
#             if red_msg is not None and red_msg.data == 1.0:
#                 current_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#                 if current_distance - self.last_red_line_distance >= self.MIN_DISTANCE_BETWEEN_RED:
#                     self.red_line_count += 1
#                     self.last_red_line_distance = current_distance
#                     rospy.loginfo(f"Red line {self.red_line_count} detected at ~3 cm (intersection)")

#                     if self.red_line_count == 1:
#                         self.stop()
#                         rospy.loginfo("First red line: Stopping and turning right")
#                         self.turn_right()
#                         self.move_forward(self.FORWARD_DISTANCE)
#                         if not self.wait_for_lanes():
#                             return False
#                     elif self.red_line_count == 2:
#                         self.stop()
#                         rospy.sleep(1.0)  # Brief pause
#                         rospy.loginfo("Second red line: Continuing straight")
#                         self.move_forward(self.FORWARD_DISTANCE)
#                         if not self.wait_for_lanes():
#                             return False
#                     elif self.red_line_count == 3:
#                         self.stop()
#                         rospy.loginfo("Third red line: Stopping and turning left")
#                         self.turn_left()
#                         self.move_forward(self.FORWARD_DISTANCE)
#                         if not self.wait_for_lanes():
#                             return False
#                     elif self.red_line_count == 4:
#                         self.stop()
#                         rospy.loginfo("Fourth red line: Stopping completely")
#                         return False
                
#                 # Wait until red line is no longer detected
#                 while not rospy.is_shutdown():
#                     try:
#                         red_msg = rospy.wait_for_message(f"/{self.vehicle_name}/red_line", Float64, timeout=1.0)
#                         if red_msg.data == 0.0:
#                             break
#                     except rospy.ROSException:
#                         break
#                     rate.sleep()

#             if yellow_msg is not None and white_msg is not None:
#                 lane_center = (yellow_msg.data + white_msg.data) / 2
#                 image_center = 320  # Assuming 640x480 image
#                 error = image_center - lane_center
#                 self.pid_control(error)
#             else:
#                 cmd = Twist2DStamped(v=self.VELOCITY / 1.5, omega=0.0)  # Move straight slowly
#                 self.pub_cmd.publish(cmd)
#                 rospy.logwarn("Lanes not detected, moving straight slowly")
                    
#             rate.sleep()

#     def run(self):
#         rospy.sleep(1)
#         distance_to_travel = rospy.get_param("~distance", 10.0)
#         while not rospy.is_shutdown() and self.red_line_count < 4:
#             result = self.lane_follow(distance_to_travel)
#             if not result and self.red_line_count == 4:
#                 break  # Exit after fourth red line
#             elif result:
#                 break  # Target distance reached

#     def on_shutdown(self):
#         self.stop()
#         rospy.loginfo("Lane following node shutting down.")
#         super(LaneFollowing, self).on_shutdown()

#     def signal_handler(self, sig, frame):
#         rospy.loginfo("Ctrl+C detected, shutting down...")
#         self.on_shutdown()
#         sys.exit(0)

# if __name__ == '__main__':
#     try:
#         node = LaneFollowing(node_name="lane_following_node")
#         node.run()
#     except rospy.ROSInterruptException:
#         pass


############## first lane following with red line 

# #!/usr/bin/env python3

# import os
# import math
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from std_msgs.msg import ColorRGBA, Float64
# from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
# from sensor_msgs.msg import CompressedImage, CameraInfo
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import signal
# import sys

# class LaneFollowing(DTROS):
    
#     def __init__(self, node_name="lane_following_node"):
#         # Initialize the DTROS node
#         super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')  # Fallback if env var not set
        
#         # Lane detection variables
#         self.bridge = CvBridge()
#         self.camera_matrix = None
#         self.distortion_coeffs = None

#         # Encoder variables
#         self.last_left_ticks = None
#         self.last_right_ticks = None
#         self._left_distance_traveled = 0.0
#         self._right_distance_traveled = 0.0
        
#         # Add red color ranges for detection
#         self.red_lower1 = np.array([0, 100, 100], np.uint8)
#         self.red_upper1 = np.array([10, 255, 255], np.uint8)
#         self.red_lower2 = np.array([170, 100, 100], np.uint8)
#         self.red_upper2 = np.array([180, 255, 255], np.uint8)

#         # Red line distance estimation
#         self.RED_LINE_Y_THRESHOLD = 200  # Approximate y-pixel for 5 cm in cropped 320x240 image
#         self.RED_LINE_DISTANCE = 0.03  # Target distance in meters (5 cm)
#         self.CAMERA_HEIGHT = 0.10  # Approximate camera height in meters
#         self.FOCAL_LENGTH = 50.0  # Approximate focal length in pixels (from camera_matrix fx)

#         # Parameters
#         self.TICKS_PER_REV = 135
#         self.WHEEL_RADIUS = 0.0318
#         self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
#         self.BASELINE = 0.077
#         self.VELOCITY = 0.2
#         self.OMEGA_SPEED = 2.5
#         self.angular_vel = 2.6
        
#         # Control parameters
#         self.KP = 0.015  # Proportional gain
#         self.KI = 0.0001  # Integral gain
#         self.KD = 0.01    # Derivative gain
#         self.TARGET_DISTANCE = 20  # meters
        
#         # Variables for PID terms
#         self.prev_error = 0.0
#         self.integral = 0.0
#         self.prev_time = None

#         self.yellow_lower = np.array([20, 100, 100], np.uint8)
#         self.yellow_upper = np.array([30, 255, 255], np.uint8)
#         self.white_lower = np.array([0, 0, 200], np.uint8)
#         self.white_upper = np.array([180, 30, 255], np.uint8)

#         self.cross_detected = 0
        
#         # Publishers
#         twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
#         self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
#         self.yellow_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/yellow_lane", Float64, queue_size=1)
#         self.white_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/white_lane", Float64, queue_size=1)
#         self.corss_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/corss_line_detect", Float64, queue_size=1)
#         self.color = rospy.Publisher(f"/{self.vehicle_name}/color", Float64, queue_size=1)
#         self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)

#         # Publisher for red line detection
#         self.red_line_pub = rospy.Publisher(f"/{self.vehicle_name}/red_line", Float64, queue_size=1)

#         # Subscribers
#         self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
#         self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
#         self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
#         self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        
#         self.sub_left_enc = rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
#         self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)
#         self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
#         self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)

#         # Handle Ctrl+C gracefully
#         signal.signal(signal.SIGINT, self.signal_handler)

#     def cb_camera_info(self, msg):
#         """Set camera parameters from CameraInfo message."""
#         self.camera_matrix = np.array(msg.K).reshape(3, 3)
#         self.distortion_coeffs = np.array(msg.D)
#         # rospy.loginfo("Camera info received and set.")

#     def cb_camera(self, msg):
#         if self.camera_matrix is None or self.distortion_coeffs is None:
#             rospy.logwarn("Camera parameters not yet received, skipping image processing.")
#             return
            
#         # Process image for lane detection and visualization
#         image = self.bridge.compressed_imgmsg_to_cv2(msg)
#         undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
#         undistorted_image = cv2.GaussianBlur(undistorted_image, (5, 5), 0)
        
#         # Crop the image (e.g., lower half of 640x480 image)
#         height, width = undistorted_image.shape[:2]
#         crop_top = height // 2
#         crop_bottom = height
#         crop_left = 0
#         crop_right = width
#         cropped_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
        
#         # Detect lanes and mark centers on the cropped image
#         yellow_pos, white_pos, processed_image = self.detect_lanes(cropped_image)
        
#         # Adjust lane positions to account for cropping offset
#         if yellow_pos is not None:
#             yellow_pos += crop_left
#         if white_pos is not None:
#             white_pos += crop_left
        
#         # Publish the processed cropped image
#         undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
#         # self.image_pub.publish(undistorted_msg)
        
#         # Publish lane detection results
#         if yellow_pos is not None:
#             self.yellow_lane_pub.publish(Float64(yellow_pos))
#         if white_pos is not None:
#             self.white_lane_pub.publish(Float64(white_pos))

#     def detect_lanes(self, image):
#         # Convert to HSV
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
#         yellow_center = None
#         white_center = None
#         red_detected = 0.0  # 0.0 means no red line or not at target distance, 1.0 means red line at ~5 cm

#         # Detect yellow lane
#         yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
#         yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         centers = []
#         for contour in yellow_contours:
#             if cv2.contourArea(contour) > 300:
#                 M = cv2.moments(contour)
#                 if M["m00"] != 0:
#                     cx = int(M["m10"] / M["m00"])
#                     cy = int(M["m01"] / M["m00"])
#                     centers.append((cx, cy))
#                     # cv2.circle(image, (cx, cy), 3, (0, 255, 255), -1)

#         if centers:
#             avg_cx = int(sum(pt[0] for pt in centers) / len(centers))
#             avg_cy = int(sum(pt[1] for pt in centers) / len(centers))
#             yellow_center = (avg_cx, avg_cy)
#             yellow_pos = avg_cx
#             cv2.circle(image, yellow_center, 5, (0, 255, 255), -1)
#         else:
#             yellow_pos = None
#             yellow_center = None

#         # Detect white lane
#         white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
#         white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         white_pos = None
#         white_y_value = None
#         for contour in white_contours:
#             if cv2.contourArea(contour) > 300:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 center_white = x + w // 2
#                 if yellow_pos is None or center_white > yellow_pos:
#                     white_pos = center_white
#                     white_y_value = int(y + h // 2)
#                     white_center = (center_white, white_y_value)
#                     cv2.circle(image, white_center, 5, (255, 255, 255), -1)
#                     break
#                 else:
#                     white_pos = yellow_pos + 500

#         if yellow_center is not None and white_pos is None:
#             white_pos = yellow_pos + 500

#         # Detect red line
#         red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
#         red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
#         red_mask = cv2.bitwise_or(red_mask1, red_mask2)
#         red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
#         for contour in red_contours:
#             if cv2.contourArea(contour) > 500:  # Adjust threshold as needed
#                 x, y, w, h = cv2.boundingRect(contour)
#                 # Use bottom edge of the red line for distance estimation
#                 y_bottom = y + h
#                 # Approximate distance using pinhole camera model: Z = f * H / y
#                 # where f is focal length, H is camera height, y is pixel offset from principal point
#                 image_height = image.shape[0]  # Cropped image height (e.g., 240)
#                 principal_point_y = image_height / 2  # Assuming principal point at image center
#                 pixel_offset_y = y_bottom - principal_point_y
#                 if pixel_offset_y > 0:  # Ensure valid pixel offset
#                     distance = (self.FOCAL_LENGTH * self.CAMERA_HEIGHT) / pixel_offset_y
#                     rospy.loginfo(f"Distance to red line: {distance}")
#                     # Check if distance is approximately 5 cm (with tolerance, e.g., 4-6 cm)
#                     if 0.03 <= distance <= 0.05:
#                         red_detected = 1.0
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
#                         cv2.putText(image, f"Red Line: {distance:.2f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#                     else:
#                         # Draw red line but don't trigger stop
#                         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
#                         cv2.putText(image, f"Red Line: {distance:.2f}m", (x, y - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
#                 break

#         # Draw lane center
#         if yellow_center is not None and white_center is not None:
#             lane_center_x = (yellow_center[0] + white_center[0]) // 2
#             lane_center_y = (yellow_center[1] + white_center[1]) // 2
#             lane_center = (lane_center_x, lane_center_y)
#             cv2.circle(image, lane_center, 5, (0, 0, 255), -1)
#             cv2.putText(image, f"Line Center: {lane_center}", (lane_center_x + 10, lane_center_y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         # Draw screen center
#         h, w = image.shape[:2]
#         screen_center = (w // 2, h // 2)
#         cv2.circle(image, screen_center, 5, (255, 0, 0), -1)
#         cv2.putText(image, f"Screen Center: {screen_center}", (screen_center[0] + 10, screen_center[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#         # Publish red line detection
#         self.red_line_pub.publish(Float64(red_detected))

#         return yellow_pos, white_pos, image

#     def cb_left_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_left_ticks is None:
#             self.last_left_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_left_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_left_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._left_distance_traveled += distance

#     def cb_right_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_right_ticks is None:
#             self.last_right_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_right_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_right_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._right_distance_traveled += distance
    
#     def stop(self):
#         msg = Twist2DStamped(v=0.0, omega=0.0)
#         self.pub_cmd.publish(msg)

#     def pid_control(self, error):
#         current_time = rospy.get_time()
#         dt = current_time - self.prev_time if self.prev_time is not None and current_time > self.prev_time else 0.001
#         self.integral += error * dt
#         error_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
#         omega = (self.KP * error) + (self.KI * self.integral) + (self.KD * error_derivative)
#         omega = max(min(omega, self.OMEGA_SPEED), -self.OMEGA_SPEED)
#         cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
#         self.pub_cmd.publish(cmd)
#         self.prev_error = error
#         self.prev_time = current_time

#     def lane_follow(self, distance, rate=20):
#         rospy.loginfo(f"Starting lane following for {distance} meters with PID control...")
#         rate = rospy.Rate(rate)
#         self.prev_time = rospy.get_time()
        
#         while not rospy.is_shutdown():
#             # Check distance traveled
#             avg_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#             if avg_distance >= distance:
#                 self.stop()
#                 rospy.loginfo("Target distance reached!")
#                 return True

#             try:
#                 yellow_msg = rospy.wait_for_message(f"/{self.vehicle_name}/yellow_lane", Float64, timeout=1.0)
#                 white_msg = rospy.wait_for_message(f"/{self.vehicle_name}/white_lane", Float64, timeout=1.0)
#                 red_msg = rospy.wait_for_message(f"/{self.vehicle_name}/red_line", Float64, timeout=1.0)
#             except rospy.ROSException as e:
#                 rospy.logwarn(f"Failed to get lane messages: {e}")
#                 yellow_msg = None
#                 white_msg = None
#                 red_msg = None
                
#             # Check for red line
#             if red_msg is not None and red_msg.data == 1.0:
#                 self.stop()
#                 rospy.loginfo("Red line detected, stopping!")
#                 return False  # Indicate early stop due to red line

#             # Continue lane following if no red line
#             if yellow_msg is not None and white_msg is not None:
#                 lane_center = (yellow_msg.data + white_msg.data) / 2
#                 image_center = 320  # Assuming 640x480 image
#                 error = image_center - lane_center
#                 self.pid_control(error)
#             else:
#                 cmd = Twist2DStamped(v=self.VELOCITY/1.5, omega=self.OMEGA_SPEED*2)
#                 self.pub_cmd.publish(cmd)
#                 rospy.logwarn("Lanes not detected, moving straight slowly")
                    
#             rate.sleep()

#     def run(self):
#         """Main execution method."""
#         rospy.sleep(1)  # Wait for subscribers to connect
#         distance_to_travel = rospy.get_param("~distance", 10.0)  # Configurable via ROS param
#         self.lane_follow(distance_to_travel, rate=20)

#     def on_shutdown(self):
#         """Shutdown handler."""
#         self.stop()
#         rospy.loginfo("Lane following node shutting down.")
#         super(LaneFollowing, self).on_shutdown()

#     def signal_handler(self, sig, frame):
#         """Handle Ctrl+C."""
#         rospy.loginfo("Ctrl+C detected, shutting down...")
#         self.on_shutdown()
#         sys.exit(0)

# if __name__ == '__main__':
#     try:
#         node = LaneFollowing(node_name="lane_following_node")
#         node.run()
#     except rospy.ROSInterruptException:
#         pass







############## first lane following

# #!/usr/bin/env python3

# import os
# import math
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from std_msgs.msg import ColorRGBA, Float64
# from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
# from sensor_msgs.msg import CompressedImage, CameraInfo
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# import signal
# import sys

# class LaneFollowing(DTROS):
    
#     def __init__(self, node_name="lane_following_node"):
#         # Initialize the DTROS node
#         super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')  # Fallback if env var not set
        
#         # Lane detection variables
#         self.bridge = CvBridge()
#         self.camera_matrix = None
#         self.distortion_coeffs = None

#         # Encoder variables
#         self.last_left_ticks = None
#         self.last_right_ticks = None
#         self._left_distance_traveled = 0.0
#         self._right_distance_traveled = 0.0
        
#         # Parameters
#         self.TICKS_PER_REV = 135
#         self.WHEEL_RADIUS = 0.0318
#         self.WHEEL_CIRC = 2.0 * math.pi * self.WHEEL_RADIUS
#         self.BASELINE = 0.077
#         self.VELOCITY = 0.2
#         self.OMEGA_SPEED = 2.5
#         self.angular_vel = 2.6
        
#         # Control parameters
#         self.KP = 0.015  # Proportional gain
#         self.KI = 0.0001  # Integral gain
#         self.KD = 0.01    # Derivative gain
#         self.TARGET_DISTANCE = 20  # meters
        
#         # Variables for PID terms
#         self.prev_error = 0.0
#         self.integral = 0.0
#         self.prev_time = None
        
#         # Publishers
#         twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
#         self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
#         self.yellow_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/yellow_lane", Float64, queue_size=1)
#         self.white_lane_pub = rospy.Publisher(f"/{self.vehicle_name}/white_lane", Float64, queue_size=1)
#         self.corss_line_detect_pub = rospy.Publisher(f"/{self.vehicle_name}/corss_line_detect", Float64, queue_size=1)
#         self.color = rospy.Publisher(f"/{self.vehicle_name}/color", Float64, queue_size=1)
#         self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)

#         # Subscribers
#         self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
#         self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
#         self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
#         self.camera_info_topic = f"/{self.vehicle_name}/camera_node/camera_info"
        
#         self.sub_left_enc = rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.cb_left_encoder)
#         self.sub_right_enc = rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.cb_right_encoder)
#         self.sub_camera = rospy.Subscriber(self.camera_topic, CompressedImage, self.cb_camera)
#         self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_camera_info)

#         self.yellow_lower = np.array([20, 100, 100], np.uint8)
#         self.yellow_upper = np.array([30, 255, 255], np.uint8)
#         self.white_lower = np.array([0, 0, 200], np.uint8)
#         self.white_upper = np.array([180, 30, 255], np.uint8)

#         self.cross_detected = 0

#         # Handle Ctrl+C gracefully
#         signal.signal(signal.SIGINT, self.signal_handler)

#     def cb_camera_info(self, msg):
#         """Set camera parameters from CameraInfo message."""
#         self.camera_matrix = np.array(msg.K).reshape(3, 3)
#         self.distortion_coeffs = np.array(msg.D)
#         rospy.loginfo("Camera info received and set.")

#     def cb_camera(self, msg):
#         if self.camera_matrix is None or self.distortion_coeffs is None:
#             rospy.logwarn("Camera parameters not yet received, skipping image processing.")
#             return
            
#         # Process image for lane detection and visualization
#         image = self.bridge.compressed_imgmsg_to_cv2(msg)
#         undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
#         undistorted_image = cv2.GaussianBlur(undistorted_image, (5, 5), 0)
        
#         # Crop the image (e.g., lower half of 640x480 image)
#         height, width = undistorted_image.shape[:2]
#         crop_top = height // 2
#         crop_bottom = height
#         crop_left = 0
#         crop_right = width
#         cropped_image = undistorted_image[crop_top:crop_bottom, crop_left:crop_right]
#         # Detect lanes and mark centers on the cropped image
#         yellow_pos, white_pos, processed_image = self.detect_lanes(cropped_image)
        
#         # Adjust lane positions to account for cropping offset
#         if yellow_pos is not None:
#             yellow_pos += crop_left
#         if white_pos is not None:
#             white_pos += crop_left
        
#         # Publish the processed cropped image
#         undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
#         # self.image_pub.publish(undistorted_msg)
        
#         # Publish lane detection results
#         if yellow_pos is not None:
#             self.yellow_lane_pub.publish(Float64(yellow_pos))
#         if white_pos is not None:
#             self.white_lane_pub.publish(Float64(white_pos))

#     def detect_lanes(self, image):
#         # Convert to HSV
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
#         yellow_center = None
#         white_center = None

#         # Detect yellow lane
#         yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
#         yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         centers = []
#         for contour in yellow_contours:
#             if cv2.contourArea(contour) > 300:
#                 M = cv2.moments(contour)
#                 if M["m00"] != 0:
#                     cx = int(M["m10"] / M["m00"])
#                     cy = int(M["m01"] / M["m00"])
#                     centers.append((cx, cy))
#                     # cv2.circle(image, (cx, cy), 3, (0, 255, 255), -1)

#         if centers:
#             avg_cx = int(sum(pt[0] for pt in centers) / len(centers))
#             avg_cy = int(sum(pt[1] for pt in centers) / len(centers))
#             yellow_center = (avg_cx, avg_cy)
#             yellow_pos = avg_cx
#             cv2.circle(image, yellow_center, 5, (0, 255, 255), -1)
#         else:
#             yellow_pos = None
#             yellow_center = None

#         # Detect white lane
#         white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
#         white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         white_pos = None
#         white_y_value = None
#         for contour in white_contours:
#             if cv2.contourArea(contour) > 300:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 center_white = x + w // 2
#                 if yellow_pos is None or center_white > yellow_pos:
#                     white_pos = center_white
#                     white_y_value = int(y + h // 2)
#                     white_center = (center_white, white_y_value)
#                     cv2.circle(image, white_center, 5, (255, 255, 255), -1)
#                     break
#                 else:
#                     white_pos = yellow_pos + 500

#         if yellow_center is not None and white_pos is None:
#             white_pos = yellow_pos + 500

#         # Draw lane center
#         if yellow_center is not None and white_center is not None:
#             lane_center_x = (yellow_center[0] + white_center[0]) // 2
#             lane_center_y = (yellow_center[1] + white_center[1]) // 2
#             lane_center = (lane_center_x, lane_center_y)
#             cv2.circle(image, lane_center, 5, (0, 0, 255), -1)
#             cv2.putText(image, f"Line Center: {lane_center}", (lane_center_x + 10, lane_center_y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         # Draw screen center
#         h, w = image.shape[:2]
#         screen_center = (w // 2, h // 2)
#         cv2.circle(image, screen_center, 5, (255, 0, 0), -1)
#         cv2.putText(image, f"Screen Center: {screen_center}", (screen_center[0] + 10, screen_center[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#         return yellow_pos, white_pos, image

#     def cb_left_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_left_ticks is None:
#             self.last_left_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_left_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_left_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._left_distance_traveled += distance

#     def cb_right_encoder(self, msg):
#         current_ticks = msg.data
#         if self.last_right_ticks is None:
#             self.last_right_ticks = current_ticks
#             return
#         delta_ticks = current_ticks - self.last_right_ticks
#         if delta_ticks > self.TICKS_PER_REV / 2:
#             delta_ticks -= self.TICKS_PER_REV
#         elif delta_ticks < -self.TICKS_PER_REV / 2:
#             delta_ticks += self.TICKS_PER_REV
#         self.last_right_ticks = current_ticks
#         distance = (delta_ticks / float(self.TICKS_PER_REV)) * self.WHEEL_CIRC
#         self._right_distance_traveled += distance
    
#     def stop(self):
#         msg = Twist2DStamped(v=0.0, omega=0.0)
#         self.pub_cmd.publish(msg)

#     def pid_control(self, error):
#         current_time = rospy.get_time()
#         dt = current_time - self.prev_time if self.prev_time is not None and current_time > self.prev_time else 0.001
#         self.integral += error * dt
#         error_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
#         omega = (self.KP * error) + (self.KI * self.integral) + (self.KD * error_derivative)
#         omega = max(min(omega, self.OMEGA_SPEED), -self.OMEGA_SPEED)
#         cmd = Twist2DStamped(v=self.VELOCITY, omega=omega)
#         self.pub_cmd.publish(cmd)
#         self.prev_error = error
#         self.prev_time = current_time

#     def lane_follow(self, distance, rate=20):
#         rospy.loginfo(f"Starting lane following for {distance} meters with PID control...")
#         rate = rospy.Rate(rate)
#         self.prev_time = rospy.get_time()
        
#         while not rospy.is_shutdown():
#             avg_distance = (self._left_distance_traveled + self._right_distance_traveled) / 2
#             if avg_distance >= distance:
#                 self.stop()
#                 rospy.loginfo("Target distance reached!")
#                 return True

#             try:
#                 yellow_msg = rospy.wait_for_message(f"/{self.vehicle_name}/yellow_lane", Float64, timeout=1.0)
#                 white_msg = rospy.wait_for_message(f"/{self.vehicle_name}/white_lane", Float64, timeout=1.0)
#             except rospy.ROSException as e:
#                 rospy.logwarn(f"Failed to get lane messages: {e}")
#                 yellow_msg = None
#                 white_msg = None
                
#             if yellow_msg is not None and white_msg is not None:
#                 lane_center = (yellow_msg.data + white_msg.data) / 2
#                 image_center = 320  # Assuming 640x480 image
#                 error = image_center - lane_center
#                 self.pid_control(error)
#             else:
#                 cmd = Twist2DStamped(v=self.VELOCITY/1.5, omega=self.OMEGA_SPEED*2)
#                 self.pub_cmd.publish(cmd)
#                 rospy.logwarn("Lanes not detected, moving straight slowly")
                    
#             rate.sleep()

#     def run(self):
#         """Main execution method."""
#         rospy.sleep(1)  # Wait for subscribers to connect
#         distance_to_travel = rospy.get_param("~distance", 10.0)  # Configurable via ROS param
#         self.lane_follow(distance_to_travel, rate=20)

#     def on_shutdown(self):
#         """Shutdown handler."""
#         self.stop()
#         rospy.loginfo("Lane following node shutting down.")
#         super(LaneFollowing, self).on_shutdown()

#     def signal_handler(self, sig, frame):
#         """Handle Ctrl+C."""
#         rospy.loginfo("Ctrl+C detected, shutting down...")
#         self.on_shutdown()
#         sys.exit(0)

# if __name__ == '__main__':
#     try:
#         node = LaneFollowing(node_name="lane_following_node")
#         node.run()
#     except rospy.ROSInterruptException:
#         pass