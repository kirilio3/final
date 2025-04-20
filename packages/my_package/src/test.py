#!/usr/bin/env python3
"""
Lane Following Node for Duckiebot using Visual Servoing + PID Control
Based on the MOOC Visual Lane Servoing activity
"""
import os
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped

class LaneFollower(DTROS):
    def __init__(self, node_name):
        super(LaneFollower, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']
        # Vehicle namespace
        # self.veh_name = os.environ.get('VEHICLE_NAME', '')
        # CV bridge for converting images
        self.bridge = CvBridge()
        # Subscribers and Publishers
        self.sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.image_cb, queue_size=1)
        self.pub = rospy.Publisher( f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        # Constant forward speed
        self.speed = 0.2
        # PID control parameters
        self.kp = 10
        self.ki = 0
        self.kd = 0.005
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.last_time = rospy.Time.now()
        # Image geometry placeholders
        self.width = None
        self.height = None
        self.steer_left_matrix = None
        self.steer_right_matrix = None

    def get_steer_matrices(self, shape):
        h, w = shape
        xs = np.linspace(-1, 1, w)
        W_left = np.tile(xs, (h, 1))
        W_right = -W_left
        return W_left, W_right

    def detect_lane_markings(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) // 255
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 55, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white) // 255
        return mask_yellow.astype(np.float32), mask_white.astype(np.float32)

    def compute_pid(self, error, current_time):
        # Compute time delta
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0.0:
            return 0.0
        # Integral term
        self.error_integral += error * dt
        # Derivative term
        derivative = (error - self.error_prev) / dt
        # PID output
        output = self.kp * error + self.ki * self.error_integral + self.kd * derivative
        # Save for next iteration
        self.error_prev = error
        self.last_time = current_time
        return output

    def image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.width is None:
            self.height, self.width, _ = frame.shape
            self.steer_left_matrix, self.steer_right_matrix = self.get_steer_matrices((self.height, self.width))
        left_mask, right_mask = self.detect_lane_markings(frame)
        # Compute raw steering error signal
        error = (np.sum(self.steer_left_matrix * left_mask) + np.sum(self.steer_right_matrix * right_mask))
        error /= (self.height * self.width)
        # Apply PID controller
        current_time = rospy.Time.now()
        omega = self.compute_pid(error, current_time)
        print(omega)
        print(self.speed)
        # Publish drive command
        twist = Twist2DStamped()
        twist.v = self.speed
        twist.omega = float(omega)
        self.pub.publish(twist)

    def run(self):
        rospy.spin()
    def on_shutdown(self):
        self.pub.publish(Twist2DStamped(v=0.0, omega=0.0))
        rospy.loginfo("LaneFollower node shutting down.")
        super(LaneFollower, self).on_shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # rospy.init_node('lane_follower', anonymous=False)
    node = LaneFollower(node_name='lane_follower')
    node.run()
