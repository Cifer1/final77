#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2

rgb = cv2.imread("frame0070.jpg")
hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

red_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([20, 255, 255]))
green_mask = cv2.inRange(hsv, np.array([60, 0, 0]), np.array([90, 255, 255]))

red_only = cv2.bitwise_and(rgb, rgb, mask=red_mask)
green_only = cv2.bitwise_and(rgb, rgb, mask=green_mask)

cv2.imshow("red only", red_only)
#cv2.imshow("green only", green_only)
cv2.waitKey(5000)
