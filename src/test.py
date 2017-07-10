#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

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
    img = cv2.imread(
        "/home/denise/catkin_ws/src/siskiyou/images/colony_sample.JPG")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow("figure")
    img_resized = cv2.resize(img,(0,0),fx=0.25,fy=0.25)
    im_h, im_w, _ = img_resized.shape
    cv2.imshow("figure", img_resized)
    # cv2.imshow("figure2", cv2.Canny(img_resized,120,130))
    # cv2.moveWindow("figure", 2000, 0)
    # cv2.moveWindow("figure2", 2000,im_h)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
