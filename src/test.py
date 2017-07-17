#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import siskiyouVision as vision

using_camera  = True

def listen():
    cv2.namedWindow("figure")
    cv2.namedWindow("figure2")
    cv2.namedWindow("figure3")

    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, callback)

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

    offset = 80
    v_h, v_w, _ = frame.shape
    cv2.moveWindow("figure", offset, offset)
    cv2.moveWindow("figure2", offset+v_w, offset)
    cv2.moveWindow("figure3", offset, offset+v_h+56)
    cv2.imshow("figure", frame)
    cv2.imshow("figure2", overlay)
    cv2.imshow("figure3", mask)
    cv2.waitKey(1)

def video_main():
    cap = cv2.VideoCapture(
        "/home/denise/Documents/Jason/media/pipette_focus.avi")

    if cap.isOpened():
        v_w = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
        v_h = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))

        offset = 80
        cv2.namedWindow("figure")
        cv2.namedWindow("figure2")
        cv2.namedWindow("figure3")
        cv2.moveWindow("figure", offset, offset)
        cv2.moveWindow("figure2", offset+v_w, offset)
        cv2.moveWindow("figure3", offset, offset+v_h+56)

        while True:
            ret, frame = cap.read()
            if ret is not True:
                break

            (corner, (mask, cnt_mask, edges)) = vision.find_tip(frame, True)

            overlay = vision.draw_mask(frame, edges, (0, 0, 255))
            overlay = vision.draw_mask(overlay, cnt_mask, (0,255,0))

            if corner is not None:
                cv2.circle(frame, corner, 2, (255,255,255), -1)
                cv2.circle(overlay, corner, 2, (255,255,255), -1)

            cv2.imshow("figure", frame)
            cv2.imshow("figure2", overlay)
            cv2.imshow("figure3", mask)
            
            cv2.waitKey(1)
        
        print "Finished!"
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print "Problem with video"

if __name__ == "__main__":
    if using_camera:
        bridge = CvBridge()
        listen()
    else:
        video_main()