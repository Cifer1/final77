#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
from red_light_green_light.msg import BlobDetections
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped


class ParkingController:
    def __init__(self):
        self.move = AckermannDriveStamped()
        self.move.drive.speed = 1
        self.move.header.stamp = rospy.Time.now()
	self.pub_move = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.sub_blob = rospy.Subscriber("/blobs", BlobDetections, self.cbBlobs, queue_size=1)
        
        self.kp = .05
        self.ki = .01
        self.kd = .005
        self.prev_error = 0
        self.prev_time = time.clock()
        
        rospy.loginfo("ParkingController initialized.")

    def cbBlobs(self, msg):
	rospy.loginfo("Got blobs message.")
        if not msg.sizes:  # no blobs
            self.move.drive.speed = 1
	    self.move.drive.steering_angle = 0
        else:
            closest_ind = max(enumerate(msg.sizes), key=lambda x: x[1].data)[0]
            rospy.loginfo("Blob size:  {}".format(msg.sizes[closest_ind].data))
	    if msg.sizes[closest_ind].data > .15:
                self.move.drive.speed = 0
                self.move.drive.steering_angle = 0
            else:
		self.move.drive.speed = 1
                error = .5 - msg.locations[closest_ind].x
		rospy.loginfo("Error:  {}".format(error))
                if abs(error) > .02:
                    self.move.drive.steering_angle = self.calc_pid(error)
	self.move.header.stamp = rospy.Time.now()
	self.pub_move.publish(self.move)
	rospy.loginfo("Sent move msg")

    def calc_pid(self, error):
        e_deriv = (error - self.prev_error) / (time.clock() - self.prev_time)
        e_int = (error + self.prev_error)/2 * (time.clock() - self.prev_time)
        self.prev_error = error
        self.prev_time = time.clock()
        return self.kp*error + self.kd*e_deriv + self.ki*e_int


if __name__ == "__main__":
    rospy.init_node("ParkingController")
    pc = ParkingController()
    rospy.spin()
