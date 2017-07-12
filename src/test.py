#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import time

# bridge = ''
# def callback(data):
#     cv_data = bridge.imgmsg_to_cv2(data, "bgr8")
#     cv2.imshow("figure", cv_data)
#     cv2.waitKey(1)

# def listen():
#     cv2.namedWindow("figure")

#     rospy.init_node('listen', anonymous=True)
#     rospy.Subscriber("/camera/image_raw", Image, callback)
#     # rospy.Subscriber("/camera/image_mono", Image, callback)
#     # rospy.Rate(10)
#     rospy.spin()

# if __name__ == "__main__":
#     bridge = CvBridge()
#     listen()

if __name__ == "__main__":
    cap = cv2.VideoCapture(
        "/home/denise/catkin_ws/src/siskiyou/media/pipette_focus.avi")

    frame = cv2.imread(
        "/home/denise/catkin_ws/src/siskiyou/media/pipette_frame.png")

    if cap.isOpened():
        v_w = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
        v_h = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))

        offset = 80
        cv2.namedWindow("figure")
        cv2.namedWindow("figure2")
        cv2.moveWindow("figure", offset, offset)
        cv2.moveWindow("figure2", offset+v_w, offset)
        
        while True:
            ret, frame = cap.read()
            if ret is not True:
                break

            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # frame = cv2.medianBlur(frame, 5)
            
            # [ignore >330%, ignore <45%, ignore >30%]
            lower = np.array([120, 0, 0])
            upper = np.array([165, 255, int(255*0.35)])
            mask = cv2.inRange(frame_hsv, lower, upper)

            # mask_gauss = cv2.GaussianBlur(mask, (5,5), 0)
            mask_median = cv2.medianBlur(mask, 5)

            cv2.imshow("figure", frame)
            cv2.imshow("figure2", mask_median)

            cv2.waitKey(1)
        
        print "Finished!"
        cv2.waitKey(0)
        cv2.destroyAllWindows()