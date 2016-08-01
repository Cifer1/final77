#!/usr/bin/env python

import numpy as np
import cv2

im = cv2.imread("frame0070.jpg")
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
#mask = cv2.inRange(hsv, np.array([50, 20, 35]), np.array([85, 155, 215]))  # green
mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 100, 200]), np.array([15, 255, 255])), cv2.inRange(hsv, np.array([180, 100, 200]), np.array([255, 255, 255])))  # red
#mask = cv2.inRange(hsv, np.array([0, 50, 130]), np.array([15, 255, 255]))
#im2 = cv2.bitwise_and(im, im, mask=mask)
#im2 = cv2.cvtColor(im2, cv2.cv.CV_BGR2GRAY)
#im2 = cv2.GaussianBlur(im2, (25, 25), 0)
#im2 = cv2.erode(im2, (5, 5), iterations=1)

im2 = cv2.bitwise_and(im, im, mask=mask)
im2 = cv2.cvtColor(im2, cv2.cv.CV_BGR2GRAY)
im2 = cv2.equalizeHist(im2)  # increase contrast

#im3 = cv2.adaptiveThreshold(im2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 0)
#cv2.imwrite("bandw.png", im3)
contours = cv2.findContours(im2, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
#im3 = cv2.adaptiveThreshold(im2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 0)
#cv2.imwrite("bandw2.png", im3)
#contours.extend(cv2.findContours(im3, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0])


contours = [c for c in contours if cv2.contourArea(c)>1000]

cv2.drawContours(im, contours, -1, (255, 255, 255), 2)

approx_contours = []
for c in contours:
    print "\nArea:  {}".format(cv2.contourArea(c))
    moments = cv2.moments(c)
    center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
    cv2.circle(im, center, 3, (255, 100, 100), 4)
    print "Moment:  ({}, {})".format(center[0], center[1])

    perim = cv2.arcLength(c, True)
    print "Perimeter:  {}".format(perim)
    approx = cv2.approxPolyDP(c, .05*perim, True)
    approx_contours.append(approx)
    print "Number of sides:  {}".format(len(approx))
cv2.drawContours(im, approx_contours, -1, (100, 255, 100), 2)

cv2.imwrite("contours.png", im)

#cv2.imshow("Contours", im)
#cv2.waitKey(0)
