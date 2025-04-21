#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, CameraInfo
import os

class LaneFollowerNode(DTROS):
    def __init__(self):
        super(LaneFollowerNode, self).__init__(node_name='lane_follower', node_type=NodeType.PERCEPTION)
        self.bridge = CvBridge()
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # PID parameters
        self.KP = 0.01
        self.KI = 0.0
        self.KD = 0.005
        self.OMEGA_MAX = 2.5
        self.VEL_BASE = 0.25
        self.prev_error = 0.0
        self.integral = 0.0
        self.deriv = 0.0
        self.error_window = deque(maxlen=5)
        self.prev_time = rospy.get_time()
        self.image_pub = rospy.Publisher(f"/{self.vehicle_name}/camera_node/image/distorted_image/compressed", CompressedImage, queue_size=10)

        # Perspective transform for bird's-eye view
        self.img_w = 640
        self.img_h = 480
        src = np.float32([[200, 480], [440, 480], [300, 300], [340, 300]])
        dst = np.float32([[0, 480], [640, 480], [0, 0], [640, 0]])
        self.M = cv2.getPerspectiveTransform(src, dst)

        # Globals for motor commands
        self.latest_v = 0.0
        self.latest_omega = 0.0

        # Subscribers
        rospy.Subscriber(f'{self.vehicle_name}/camera_node/camera_info', CameraInfo, self.cb_camera_info)
        rospy.Subscriber(f'{self.vehicle_name}/camera_node/image/compressed', CompressedImage, self.cb_image)

        # Publisher
        self.pub_cmd = rospy.Publisher(f'{self.vehicle_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        # Publish motor commands at 10 Hz
        self.rate_timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd)

    def cb_camera_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)

    def cb_image(self, msg):
        if self.camera_matrix is None:
            return
        # 1. Undistort & blur
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        undist = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)
        blur = cv2.GaussianBlur(undist, (5, 5), 0)

        # 2. Bird's-eye transform
        warped = cv2.warpPerspective(blur, self.M, (self.img_w, self.img_h))

        # 3. Color segmentation
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        mask_w = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))

        # 4. Sliding window search for left & right lanes
        left_pts = self.sliding_window(mask_y)
        right_pts = self.sliding_window(mask_w)

        # 5. Compute desired v and omega
        if left_pts and right_pts:
            v, omega = self.compute_control(left_pts, right_pts)
        else:
            v, omega = self.VEL_BASE * 0.5, 0.0

        # Save to globals
        self.latest_v = v
        self.latest_omega = omega
        undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(warped)
        self.image_pub.publish(undistorted_msg)

    def sliding_window(self, binary_img, nwindows=9, margin=100, minpix=50):
        histogram = np.sum(binary_img[self.img_h//2:, :], axis=0)
        base = np.argmax(histogram)
        nonzero = binary_img.nonzero()
        ys, xs = nonzero
        window_h = int(self.img_h / nwindows)
        x_current = base
        pts = []
        for w in range(nwindows):
            y_low = self.img_h - (w+1)*window_h
            y_high = self.img_h - w*window_h
            x_low = x_current - margin
            x_high = x_current + margin
            inds = ((ys >= y_low) & (ys < y_high) & (xs >= x_low) & (xs < x_high)).nonzero()[0]
            if len(inds) > minpix:
                x_current = int(np.mean(xs[inds]))
            for i in inds:
                pts.append((xs[i], ys[i]))
        return pts

    def compute_control(self, left_pts, right_pts):
        # Fit polynomials
        ys_l, xs_l = zip(*left_pts)
        ys_r, xs_r = zip(*right_pts)
        poly_l = np.polyfit(ys_l, xs_l, 2)
        poly_r = np.polyfit(ys_r, xs_r, 2)
        y_ref = self.img_h
        x_l = np.polyval(poly_l, y_ref)
        x_r = np.polyval(poly_r, y_ref)
        lane_center = (x_l + x_r) / 2.0
        error = lane_center - (self.img_w / 2.0)

        # PID
        now = rospy.get_time()
        dt = max(now - self.prev_time, 1e-3)
        self.error_window.append(error)
        smooth = sum(self.error_window)/len(self.error_window)
        P = self.KP * smooth
        self.integral += smooth * dt
        I = self.KI * self.integral
        D_raw = (smooth - self.prev_error)/dt
        self.deriv = 0.8*self.deriv + 0.2*D_raw
        D = self.KD * self.deriv
        omega = np.clip(P + I + D, -self.OMEGA_MAX, self.OMEGA_MAX)
        v = self.VEL_BASE
        self.prev_error = smooth
        self.prev_time = now
        return v, omega

    def publish_cmd(self, event):
        cmd = Twist2DStamped(v=self.latest_v, omega=self.latest_omega)
        self.pub_cmd.publish(cmd)

    def run(self):
        rospy.loginfo("Lane follower node started.")
        rospy.spin()

    def on_shutdown(self):
        cmd = Twist2DStamped(v=0, omega=0)
        self.pub_cmd.publish(cmd)
        self.rate_timer.shutdown()
        self.pub_cmd.unregister()
        super(LaneFollowerNode, self).shutdown()

if __name__ == '__main__':
    # rospy.init_node('lane_follower_node', anonymous=False)
    node = LaneFollowerNode()
    node.run()
    
