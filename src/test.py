#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import siskiyouVision as vision

if __name__ == "__main__":
    path = "/home/denise/Documents/Jason/media/pipette_focus.avi"
    rospy.init_node("video_publisher")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()
    r = rospy.Rate(65)
    print "publishing..."
    loop_n = 0
    while not rospy.is_shutdown():
        print "loop:", loop_n
        cap = cv2.VideoCapture(path)
        ret = True
        while True:
            ret, frame = cap.read()
            if ret is not True or rospy.is_shutdown():
                break
            # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frame_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(frame_msg)
            r.sleep()
        loop_n += 1


