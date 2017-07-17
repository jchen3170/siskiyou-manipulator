#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def find_tip(frame, visuals):   
    v_h, v_w, _ = frame.shape

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    hue_min = 120
    hue_max = 170
    sat_min = 0
    sat_max = int(255*0.50)
    val_min = 0
    val_max = int(255*0.36)

    # [120-165, 0-255, 0-255*0.38]
    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])
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

def draw_mask(img, mask, color):
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