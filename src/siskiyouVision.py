#!/usr/bin/env python

'''
Vision library to filter and edit images fed in by the camera

'''

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# function for finding the tip of a pipette.
# adjust HSV min-max values accordingly
# inputs: 
#     frame: image in BGR format, (h, w, 3)
#     visuals: boolean (if true returns additional images for display)
# outputs:
#     corner: the coordinate of the pipette tip
#     mask: mask from HSV filter
#     cnt_mask: contour mask from contour approximation
#     edge: edge map of mask
def find_tip(frame, visuals): 
    # pull heigh and width values
    v_h, v_w, _ = frame.shape

    # convert image to HSV
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # set parameters for HSV filtering
    hue_min = 120
    hue_max = 170
    sat_min = 0
    sat_max = int(255*0.50)
    val_min = 0
    val_max = int(255*0.36)

    # format parameters and filter
    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])
    mask = cv2.inRange(frame_hsv, lower, upper)

    # cleanup resulting image by reducing noise and holes
    mask_median = cv2.medianBlur(mask, 5)
    mask_close = cv2.morphologyEx(mask_median, 
        cv2.MORPH_CLOSE, np.ones((20,20),np.uint8))
    mask = mask_close.copy()

    # get edge map of cleaned image
    edges = cv2.Canny(mask_close, 100, 200)

    # approximate cleaned image using hull contours
    contours = []
    try:
        contours, _ = cv2.findContours(mask_close, cv2.RETR_TREE, 
            cv2.CHAIN_APPROX_NONE)
    except:
        _, contours, _ = cv2.findContours(mask_close, cv2.RETR_TREE, 
            cv2.CHAIN_APPROX_NONE)

    # make sure contours exist
    if len(contours) is not 0:
        cnt = contours[0]
        # loop through contours and determine largest one
        max_area = cv2.contourArea(cnt)
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                cnt = c
                max_area = area
        # approximate contour of largest one
        peri = cv2.arcLength(cnt, True)
        eps = peri * 0.045
        approx = cv2.approxPolyDP(cnt, eps, True)
        # get a mask of the contour
        cnt_mask = np.zeros((v_h,v_w,1), np.uint8)
        cv2.drawContours(cnt_mask, [approx], -1, (255), 1)

        # if contour is triangular, try to find corner (pipette tip)
        if len(approx) is 3:
            max_sum = 0
            corner = 0
            # get the point furthest from the other two
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

# draws the input mask on the input image with the input color
# inputs:
#     img: the image in BGR format (h, w, 3)
#     mask: the mask to draw with (h, w)
#     color: BGR tuple for color (B, G, R)
# outputs:
#     img: original image with mask drawn on
def draw_mask(img, mask, color):
    # make sure mask is not all 0's before proceeding
    if np.any(mask):
        h, w, _ = img.shape
        blank = np.zeros((h,w,3), np.uint8)
        n = 0
        # generate image of the specified color
        for c in color:
            blank[:,:,n] = c
            n += 1
        # color the mask using the blank
        outline = cv2.bitwise_and(blank, blank, mask=mask)
        # draw colored mask onto input image
        img_prep = cv2.bitwise_and(img, img, 
            mask=cv2.bitwise_not(mask))
        return cv2.add(img_prep, outline)
    else:
        return img