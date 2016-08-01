#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2

rgb = cv2.imread("BigSquares.png")
r = rgb[:, :, 2].ravel()  # flatten
plt.plot(r)
plt.show()
