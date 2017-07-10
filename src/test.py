#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = ''

def callback(data):
    cv_data = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("figure", cv_data)
    cv2.waitKey(1)

def callback2(data):
    cv_data = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("figure2", cv_data)
    cv2.waitKey(1)

def listen():
    cv2.namedWindow("figure")
    cv2.namedWindow("figure2")

    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, callback)
    rospy.Subscriber("/camera/image_mono", Image, callback2)
    # rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    bridge = CvBridge()
    listen()