#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). 
Assumes R232->USB and 25pin->USB

'''

import time

import siskiyouSerial
import siskiyouCommands as command
import siskiyouControls as controls
import siskiyouGetPort as port
import siskiyouLibrary as sisk
from siskiyou.msg import siskiyouPosition

def init_controls(ser):
    P = 8000 # must be between 4000 - 32000
    I = 250 # must be between 1 - 32000
    D = 1000 # must be between 1000 - 32000
    controls.setGains(axis.ALL, P, I, D, ser)

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    accel = 100
    vel = 1000
    axis = sisk.Y

    ser = siskiyouSerial.SiskiyouSerial(port)
    ser.init()

    print "status:", command.getStatus(axis, ser)

    print "init pos:", command.getPosition(axis, ser)

    command.setHome(axis, ser)
    command.relPosition(axis, ser, 50000, vel, accel)

    time.sleep(3)
    print "mid:", command.getPosition(axis, ser)
    command.returnHome(axis, ser, vel, accel)

    time.sleep(3)
    print 'final pos:', command.getPosition(axis, ser)


    ser.close()



