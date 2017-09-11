#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). (Assumes R232->USB)

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
import siskiyouGetPort

# main function 
def main():
    global PORT, CAMERA_TOPIC
    global image, global_corner, pt_stack
    global bridge, ser
    global pos_control, vel_control

    # (NOT TUNED) control method (set one to "True")
    pos_control = False
    vel_control = True

    # assigned USB port address for device (change accordingly)
    PORT = siskiyouGetPort.getPort()
    if PORT is '':
        PORT = "/dev/ttyUSB0"
    else:
        PORT = "/dev/" + PORT

    # camera publish ROS topic name
    CAMERA_TOPIC = "/camera/image_raw"

    # global scope variables 
    image = np.zeros((480,760,3), np.uint8)
    global_corner = (0,0)
    pt_stack = []

    # intialize opencv image converter
    bridge = CvBridge()
    # initialize serial port class
    ser = siskiyouSerial.SiskiyouSerial(PORT)
    # intialize GUI class
    gui = siskiyouGUI.Window(ser)

    # initialize subscriber
    rospy.init_node("siskiyouMain", anonymous=False)
    rospy.Subscriber(CAMERA_TOPIC, Image, 
        lambda data: callback(data, gui), queue_size=10)

    # begin mainloop
    main_loop(ser, gui)

    # close serial connection on exit
    ser.close()

# main loop that updates GUI and sends commands
def main_loop(ser, gui):
    global image, pt_stack

    pos = (0,0,0)
    mov = (False,False,False)
    lims = (False,False,False)
    status = ('0','0','0')

    while True:
        # grab position / status of all axis
        pos = com.getPositionAll(ser)
        mov, lims, status = com.getStatusAll(ser)

        # update GUI variables
        gui.setPosition(pos)
        gui.setMoving(mov)
        gui.setLimits(lims)
        gui.setStatus(status)
        pt_stack = gui.getImagePoints()
        movePipette(gui)
        gui.setImage(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        # update GUI visuals
        flag = gui.update()

        # break if GUI is closed or script is stopped
        if rospy.is_shutdown() or flag:
            break

        # 1 ms delay between updates
        time.sleep(0.001)

# callback function for rospy subscriber: analyzes the camera feed
def callback(data, gui):
    global image, global_corner, bridge, pt_stack

    # convert from ROS image format to CV2 image format
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    mask = np.zeros(frame.shape, np.uint8)
    cnt_mask = np.zeros(frame.shape, np.uint8)
    edges = np.zeros(frame.shape, np.uint8)

    # find pipette tip in image
    (corner, visual_tuple) = vision.find_tip(frame, True)

    # check if filter was successful
    if visual_tuple is not None:
        if visual_tuple[0] is not None:
            mask = visual_tuple[0]
        if visual_tuple[1] is not None:
            cnt_mask = visual_tuple[1]
        if visual_tuple[2] is not None:
            edges = visual_tuple[2]
        overlay_edges = vision.draw_mask(frame, edges, (0, 0, 255))
        overlay_contour = vision.draw_mask(frame, cnt_mask, (0,255,0))
        overlay_both = vision.draw_mask(overlay_edges, cnt_mask, (0,255,0))
    # otherwise just return the original frame
    else:
        overlay = frame
        overlay_edges = frame
        overlay_contour = frame
        overlay_both = frame

    # if pipette tip was found, draw onto resulting image
    if corner is not None:
        cv2.circle(frame, corner, 2, (255,255,255), -1)
        cv2.circle(overlay_edges, corner, 2, (255,255,255), -1)
        cv2.circle(overlay_contour, corner, 2, (255,255,255), -1)
        cv2.circle(overlay_both, corner, 2, (255,255,255), -1)
        global_corner = corner
    else:
        cv2.circle(frame, global_corner, 2, (255,255,255), -1)
        cv2.circle(overlay_edges, global_corner, 2, (255,255,255), -1)
        cv2.circle(overlay_contour, global_corner, 2, (255,255,255), -1)
        cv2.circle(overlay_both, global_corner, 2, (255,255,255), -1)

    # use GUI flags to determine which visuals to show
    edge_flag = gui.getEdgeFlag()
    contour_flag = gui.getContourFlag()

    if edge_flag and contour_flag:
        image = overlay_both
    elif edge_flag:
        image = overlay_edges
    elif contour_flag:
        image = overlay_contour
    else:
        image = frame

    # draw selected points
    for pt in pt_stack:
        cv2.circle(image, pt, 2, (0,200,0), -1)

# function to move pipette tip to selected points
def movePipette(gui):
    global image, global_corner, ser, pt_stack, pos_control, vel_control

    if pt_stack:
        # cv2.line(image, global_corner, pt_stack[0], (200,100,0))
        if gui.getMoveFlag():
            # scale velocity values based on distance (not yet implemented)
            x_diff = pt_stack[0][0]-global_corner[0]
            y_diff = pt_stack[0][1]-global_corner[1]

            # determine if set to position or velocity control
            if pos_control:
                # determine gains
                counts_per_pixel = 1
                x_amt = counts_per_pixel * x_diff
                y_amt = counts_per_pixel * y_diff
                com.moveRelative(sisk.X, ser, x_amt, gui.SP, gui.AC)
                com.moveRelative(sisk.Y, ser, y_amt, gui.SP, gui.AC)
            elif vel_control:
                # determine gains
                k = 1
                x_vel = k * x_diff
                y_vel = -k * y_diff
                com.velocityMode(sisk.X, ser, x_vel, gui.AC)
                com.velocityMode(sisk.Y, ser, y_vel, gui.AC)
            
    else:
        if gui.getMoveFlag():
            gui.setMoveFlag(False)


if __name__ == "__main__":
    main()