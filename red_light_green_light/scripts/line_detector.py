#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from red_light_green_light.msg import BlobDetections
from std_msgs.msg import ColorRGBA, Float64
from geometry_msgs.msg import Point
import math


class LineDetector:
    def __init__(self):
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/lines", BlobDetections, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("LineDetector initialized.")

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False): return
        im = self.bridge.imgmsg_to_cv2(image_msg)
        im = im[.9*len(im):]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        self.msg = BlobDetections()
        self.find_color(im, hsv, (50, 255, 255), np.array([20, 85, 185]), np.array([35, 105, 205]))  # yellowish tape
        self.pub_image.publish(self.msg)
        self.thread_lock.release()

    def find_color(self, im, hsv, label_color, min_thresh, max_thresh):
        mask = cv2.inRange(hsv, min_thresh, max_thresh)
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        #cv2.drawContours(im, contours, -1, (255, 255, 255), 2)
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 10: continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05*perim, True)
            #if len(approx) == 4 and len(cv2.convexityDefects(c, cv2.convexHull(c))) <= 1:
            if len(approx) == 4:
                approx_contours.append(approx)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                cv2.circle(im, center, 3, (255, 100, 100), 4)
                print "Moment:  ({}, {})".format(center[0], center[1])

                msg_color = ColorRGBA()
                msg_color.r, msg_color.g, msg_color.b = label_color
                self.msg.colors.append(msg_color)
                print "Label color:  {}".format(label_color)
                msg_size = Float64()
                #msg_size.data = max(math.sqrt((approx[1][0][0]-approx[0][0][0])**2+(approx[1][0][1]-approx[0][0][1])**2), math.sqrt((approx[2][0][0]-approx[1][0][0])**2+(approx[2][0][1]-approx[2][0][0])**2))
                msg_size.data = float((max(approx, key=lambda x: x[0][0])[0][0] - min(approx, key=lambda x: x[0][0])[0][0])) / len(im[0])
                print "Width:  {}".format(msg_size.data)
                self.msg.sizes.append(msg_size)
                msg_loc = Point()
                msg_loc.x, msg_loc.y = float(center[0]) / len(im[0]), float(center[1]) / len(im)
                self.msg.locations.append(msg_loc)


if __name__ == "__main__":
    rospy.init_node("LineDetector")
    bd = LineDetector()
    rospy.spin()
