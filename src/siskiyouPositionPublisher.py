#!/usr/bin/env python

import sys
import serial
import rospy

import siskiyouGetPort as gp
import siskiyouMotion as motion
import siskiyouAxis as axis
from siskiyou.msg import siskiyouPosition

def init():
    ser = serial.Serial()
    ser.baudrate = 38400
    ser.port = port.getPort();
    ser.timeout = 1
    return ser

def publish(ser):
    pos = siskiyouPosition()
    pub = rospy.Publisher("siskiyouPosition", siskiyouPosition, queue_size=10)
    rospy.init_node('siskiyouPosition', anonymous=True)
    r = rospy.Rate(10)
    ser.open()
    while not rospy.is_shutdown():
        pos.X = motion.getPosition(axis.X, ser)
        pos.Y = motion.getPosition(axis.Y, ser)
        pos.Z = motion.getPosition(axis.Z, ser)
        pos.T = motion.getPosition(axis.T, ser)
        pub.publish(pos)
        r.sleep()
    ser.close()

if __name__ == "__main__":
    ser = init()
    publish(ser)