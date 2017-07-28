#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). 
Assumes R232->USB and 25pin->USB

'''
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import siskiyouSerial
import siskiyouCommands as com
import siskiyouLibrary as sisk
import siskiyouVision as vision
import siskiyouGUI
# import siskiyouGetPort as port

image = np.zeros((480,760,3), np.uint8)
def main_loop(ser, gui):
    rospy.init_node("siskiyouMain", anonymous=False)
    rospy.Subscriber("/camera/image_raw", Image, callback, queue_size=10)

    pos = (0,0,0)
    mov = (False,False,False)
    lims = (False,False,False)
    status = ('0','0','0')

    while True:
        pos = com.getPositionAll(ser)
        mov, lims, status = com.getStatusAll(ser)

        gui.setPosition(pos)
        gui.setMoving(mov)
        gui.setLimits(lims)
        gui.setStatus(status)
        for pt in gui.getImagePoints():
            cv2.circle(image, pt, 2, (0,200,0), -1)
        gui.setImage(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        flag = gui.update()

        if rospy.is_shutdown() or flag:
            break

        time.sleep(0.001)

def callback(data):
    global image

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

    image = cv2.resize(frame, (0,0), fx=0.75, fy=0.75)

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    vel = 1000
    accel = 100
    dist = 2000000

    bridge = CvBridge()
    ser = siskiyouSerial.SiskiyouSerial(port)
    gui = siskiyouGUI.Window(ser)

    main_loop(ser, gui)

    ser.close()