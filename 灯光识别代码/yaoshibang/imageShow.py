import cv2
import cv2
from scipy.special import y1_zeros

from ultralytics import YOLO
import os
import random
import pyrealsense2 as rs
import numpy as np

# 读取图像
img = cv2.imread('output1.jpg')
cv2.imshow('sdasd',img)
cv2.waitKey(0)
