#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). 
Assumes R232->USB and 25pin->USB

'''

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import siskiyouSerial
import siskiyouCommands as command
import siskiyouLibrary as sisk
import siskiyouVision as vision
# import siskiyouGetPort as port

def main_loop():
    cv2.namedWindow("figure")
    cv2.namedWindow("figure2")
    cv2.namedWindow("figure3")

    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, callback)

    # while not rospy.is_shutdown():

    
    rospy.spin()

def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    (corner, visual_tuple) = vision.find_tip(frame, True)

    mask = np.zeros(frame.shape, np.uint8)
    cnt_mask = np.zeros(frame.shape, np.uint8)
    edges = np.zeros(frame.shape, np.uint8)

    if visual_tuple is not None:
        mask = visual_tuple[0]
        cnt_mask = visual_tuple[1]
        edges = visual_tuple[2]
        overlay = vision.draw_mask(frame, edges, (0, 0, 255))
        overlay = vision.draw_mask(overlay, cnt_mask, (0,255,0))
    else:
        overlay = frame

    if corner is not None:
        cv2.circle(frame, corner, 2, (255,255,255), -1)
        cv2.circle(overlay, corner, 2, (255,255,255), -1)

    cv2.imshow("figure", frame)
    cv2.imshow("figure2", overlay)
    cv2.imshow("figure3", mask)

    offset = 80
    v_h, v_w, _ = frame.shape
    cv2.moveWindow("figure", offset, offset)
    cv2.moveWindow("figure2", offset+v_w, offset)
    cv2.moveWindow("figure3", offset, offset+v_h+56)

    cv2.waitKey(1)

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    vel = 1000
    accel = 100
    dist = 2000000

    bridge = CvBridge()
    # ser = siskiyouSerial.SiskiyouSerial(port)
    # ser.close()

    main_loop()

    