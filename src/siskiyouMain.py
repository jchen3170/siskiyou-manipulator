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
    P = 4000 # must be between 4000 - 32000
    I = 1 # must be between 1 - 32000
    D = 1000 # must be between 1000 - 32000
    controls.setGains(axis.ALL, P, I, D, ser)

if __name__ == "__main__":
    port = "/dev/ttyUSB0"

    ser = siskiyouSerial.SiskiyouSerial(port)
    ser.init()

    print "status", command.getStatus(sisk.X, ser)

    print "Init pos:", command.getPosition(sisk.X, ser)

    # command.relPosition(sisk.X, 75000, 3000, 100, ser)
    # command.returnHome(sisk.X, 3000, 100, ser)

    # time.sleep(0.5)
    # print command.getPosition(sisk.X, ser)

    ser.close()



