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
        "/home/denise/Documents/Jason/media/pipette_focus.avi")

    frame = cv2.imread(
        "/home/denise/Documents/Jason/media/pipette_frame.png")

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

            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # [120-165, 0-255, 0-255*0.38]
            lower = np.array([120, 0, 0])
            upper = np.array([170, int(255*0.50), int(255*0.36)])
            mask = cv2.inRange(frame_hsv, lower, upper)

            # mask_gauss = cv2.GaussianBlur(mask, (5,5), 0)
            mask_median = cv2.medianBlur(mask, 5)
            mask_close = cv2.morphologyEx(mask_median, 
                cv2.MORPH_CLOSE, np.ones((20,20),np.uint8))
            mask = mask_close.copy()

            edges = cv2.Canny(mask_close, 100, 200)
            line = np.zeros((v_h,v_w,3), np.uint8)
            line[:,:,2] = 255
            outline = cv2.bitwise_and(line, line, mask=edges)
            frame_prep = cv2.bitwise_and(frame, frame, 
                mask=cv2.bitwise_not(edges))
            overlay = cv2.add(frame_prep, outline)

            contours , _ = cv2.findContours(mask_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
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

            line2 = np.zeros((v_h,v_w,3), np.uint8)
            line2[:,:,1] = 255
            outline2 = cv2.bitwise_and(line2,line2,mask=cnt_mask)
            frame_prep = cv2.bitwise_and(overlay,overlay,
                mask=cv2.bitwise_not(cnt_mask))
            overlay2 = cv2.add(frame_prep, outline2)

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
                cv2.circle(overlay2, corner, 2, (255,255,255), -1)
                cv2.circle(frame, corner, 2, (255,255,255), -1)

            cv2.imshow("figure", frame)
            cv2.imshow("figure2", overlay2)
            cv2.imshow("figure3", mask)
            
            cv2.waitKey(1)
        
        print "Finished!"
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print "Problem with video"