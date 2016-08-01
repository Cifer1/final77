#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading


class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image", Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False): return
        im = self.bridge.imgmsg_to_cv2(image_msg)
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        #self.find_color(im, (200, 200, 255), cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 100, 200]), np.array([15, 255, 255])), cv2.inRange(hsv, np.array([160, 100, 200]), np.array([255, 255, 255]))))  # red
        self.find_color(im, (200, 200, 255), cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 110, 180]), np.array([15, 255, 255])), cv2.inRange(hsv, np.array([175, 110, 180]), np.array([180, 255, 255]))))  # red
	#self.find_color(im, (200, 255, 200), cv2.inRange(hsv, np.array([50, 175, 115]), np.array([70, 240, 175])))  # green
        self.find_color(im, (150, 255, 255), cv2.inRange(hsv, np.array([40, 55, 140]), np.array([85, 185, 250])))  # green
	#self.find_color(im, (100, 255, 255), cv2.inRange(hsv, np.array([20, 230, 200]), np.array([35, 255, 230])))  # yellow
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))
        except CvBridgeError as e:
            print e
        self.thread_lock.release()

    def find_color(self, im, label_color, mask):
        #mask = cv2.inRange(hsv, min_thresh, max_thresh)
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        #cv2.drawContours(im, contours, -1, (255, 255, 255), 2)
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 200: continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05*perim, True)
            #if len(approx) == 4 and len(cv2.convexityDefects(c, cv2.convexHull(c))) <= 1:
            if len(approx) == 4:
                approx_contours.append(approx)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                cv2.circle(im, center, 3, (255, 100, 100), 4)
                print "Moment:  ({}, {})".format(center[0], center[1])

                print "Number of sides:  {}".format(len(approx))
        cv2.drawContours(im, approx_contours, -1, label_color, 2)


if __name__ == "__main__":
    rospy.init_node("Echo")
    e = Echo()
    rospy.spin()
