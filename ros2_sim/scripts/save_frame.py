#!/usr/bin/env python3
"""Save the next frame of an image topic to a PNG:  save_frame.py out.png [topic=/shot/image]"""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

rclpy.init()
node = Node("save_frame")
got = []
node.create_subscription(Image, sys.argv[2] if len(sys.argv) > 2 else "/shot/image", got.append, 1)
while not got:
    rclpy.spin_once(node, timeout_sec=0.5)
for _ in range(3):      # let the picture settle
    got.clear()
    while not got:
        rclpy.spin_once(node, timeout_sec=0.5)
cv2.imwrite(sys.argv[1], cv2.cvtColor(CvBridge().imgmsg_to_cv2(got[0], "rgb8"), cv2.COLOR_RGB2BGR))
