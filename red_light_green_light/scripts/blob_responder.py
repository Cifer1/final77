#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from red_light_green_light.msg import BlobDetections
from std_msgs.msg import Float64
from geometry_msgs.msg import ColorRGBA, Point
from ackermann_msgs.msg import AckermannDriveStamped


class BlobResponder:
    def __init__(self):
        self.move = AckermannDriveStamped()
        self.move.drive.speed = 2
        self.pub_move = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.sub_blob = rospy.Subscriber("/Blobs", BlobDetections, self.cbBlobs, queue_size=1)
        rospy.loginfo("BlobResponder initialized.")

    def cbBlobs(self, msg):
        if not msg.colors:  # nothing
            self.move.drive.speed = 2
        else:
            closest_rgba = msg.colors[max(enumerate(msg.sizes), key=lambda x: x[1])[0]]
            closest_color = (closest_rgba.r, closes_rgba.g, closest_rgba.b)
            if closest_color == (150, 150, 255):  # red
                self.move.drive.speed = 0
            elif closest_color == (50, 255, 255):  # yellow
                self.move.drive.speed = .5
            else:  # green
                self.move.drive.speed = 2
        self.pub_move.publish(self.move)


if __name__ == "__main__":
    rospy.init_node("BlobResponder")
    br = BlobResponder()
    rospy.spin()
