#!/usr/bin/env python

import numpy as np
import cv2

im = cv2.imread("frame0070.jpg")
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
green_mask = cv2.inRange(hsv, np.array([60, 0, 0]), np.array([90, 255, 255]))
#red_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([30, 255, 255]))

#cv2.imwrite("frame0070_green", green_mask)
#im = cv2.imread("frame0070_green", cv2.COLOR_LOAD_IMAGE_GRAYSCALE)
#im = cv2.bitwise_and(im, im, mask=green_mask)
#im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
im = 255 - green_mask

params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 0;
params.maxThreshold = 256;
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = True
params.minCircularity = 0
params.filterByConvexity = True
params.minConvexity = 0
params.filterByInertia = True
params.minInertiaRatio = 0

detector = cv2.SimpleBlobDetector()
keypoints = detector.detect(im)
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
