#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


def echo_callback(msg):
    zed_pub.publish(msg.data)


rospy.init_node("echo")

zed_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, echo_callback)
zed_pub = rospy.Publisher("image_echo", Image)

rospy.spin()
