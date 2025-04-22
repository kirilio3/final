#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, Float64

if __name__ == '__main__':
    rospy.init_node('led_publisher')

    led_pub = rospy.Publisher('/duckiebot/led_emitter_node/led_pattern', LEDPattern, queue_size=1)

    color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue LEDs
    pattern = LEDPattern()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        led_pub.publish(pattern)
        rate.sleep()

