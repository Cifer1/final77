#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2

rgb = cv2.imread("BigSquares.png")
hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
h = hsv[:, :, 0].ravel()
hist = plt.hist(h, 100)
plt.show()
