#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import time

using_camera  = False

bridge = ''
def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    (corner, visual_tuple) = find_tip(frame, True)

    mask = np.zeros(frame.shape, np.uint8)
    cnt_mask = np.zeros(frame.shape, np.uint8)
    edges = np.zeros(frame.shape, np.uint8)

    if visual_tuple is not None:
        mask = visual_tuple[0]
        cnt_mask = visual_tuple[1]
        edges = visual_tuple[2]
        overlay = mask_draw(frame, edges, (0, 0, 255))
        overlay = mask_draw(overlay, cnt_mask, (0,255,0))
    else:
        overlay = frame

    if corner is not None:
        cv2.circle(frame, corner, 2, (255,255,255), -1)
        cv2.circle(overlay, corner, 2, (255,255,255), -1)

    cv2.imshow("figure", frame)
    cv2.imshow("figure2", overlay)
    cv2.imshow("figure3", mask)

    cv2.waitKey(1)

def listen():
    v_h = 480
    v_w = 752
    offset = 80
    cv2.namedWindow("figure")
    cv2.namedWindow("figure2")
    cv2.namedWindow("figure3")
    cv2.moveWindow("figure", offset, offset)
    cv2.moveWindow("figure2", offset+v_w, offset)
    cv2.moveWindow("figure3", offset, offset+v_h+56)

    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, callback)
    rospy.spin()

def mask_draw(img, mask, color):
    h, w, _ = img.shape
    blank = np.zeros((h,w,3), np.uint8)
    n = 0
    for c in color:
        blank[:,:,n] = c
        n += 1
    outline = cv2.bitwise_and(blank, blank, mask=mask)
    img_prep = cv2.bitwise_and(img, img, 
        mask=cv2.bitwise_not(mask))
    return cv2.add(img_prep, outline)

def find_tip(frame, visuals):
    v_h, v_w, _ = frame.shape

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # [120-165, 0-255, 0-255*0.38]
    lower = np.array([120, 0, 0])
    upper = np.array([170, int(255*0.50), int(255*0.36)])
    mask = cv2.inRange(frame_hsv, lower, upper)

    mask_median = cv2.medianBlur(mask, 5)
    mask_close = cv2.morphologyEx(mask_median, 
        cv2.MORPH_CLOSE, np.ones((20,20),np.uint8))
    mask = mask_close.copy()

    edges = cv2.Canny(mask_close, 100, 200)

    contours , _ = cv2.findContours(mask_close, cv2.RETR_TREE, 
        cv2.CHAIN_APPROX_NONE)
    if len(contours) is not 0:
        cnt = contours[0]
        max_area = cv2.contourArea(cnt)
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                cnt = c
                max_area = area
        peri = cv2.arcLength(cnt, True)
        eps = peri * 0.045
        approx = cv2.approxPolyDP(cnt, eps, True)
        cnt_mask = np.zeros((v_h,v_w,1), np.uint8)
        cv2.drawContours(cnt_mask, [approx], -1, (255), 1)

        if len(approx) is 3:
            max_sum = 0
            corner = 0
            for pt1 in approx:
                d_sum = 0
                for pt2 in approx:
                    d_sum += np.linalg.norm(pt1-pt2)
                if d_sum > max_sum:
                    corner = pt1
                    max_sum = d_sum
            corner = tuple(map(tuple, corner))[0]
        else:
            corner = None

        if visuals:
            return (corner, (mask, cnt_mask, edges))
        else:
            return corner
    else:
        if visuals:
            return (None, None)
        else:
            return None

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

            (corner, (mask, cnt_mask, edges)) = find_tip(frame, True)

            overlay = mask_draw(frame, edges, (0, 0, 255))
            overlay = mask_draw(overlay, cnt_mask, (0,255,0))

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