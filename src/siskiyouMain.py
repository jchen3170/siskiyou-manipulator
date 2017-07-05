#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). 
Assumes R232->USB and 25pin->USB

'''

import time

import siskiyouSerial
import siskiyouCommands as command
import siskiyouGetPort as port
import siskiyouLibrary as sisk
from siskiyou.msg import siskiyouPosition

def position(ser):
    x_pos = command.getPosition(sisk.X, ser)
    y_pos = command.getPosition(sisk.Y, ser)
    z_pos = command.getPosition(sisk.Z, ser)
    return (x_pos, y_pos, z_pos)

def status(ser):
    x_status = command.getStatus(sisk.X, ser)
    y_status = command.getStatus(sisk.Y, ser)
    z_status = command.getStatus(sisk.Z, ser)
    return (x_status, y_status, z_status)

def isFinished(s):
    moving = (command.isMoving(s[0]),
              command.isMoving(s[1]),
              command.isMoving(s[2]))
    inpos = (command.isInPosition(s[0]),
             command.isInPosition(s[1]),
             command.isInPosition(s[2]))
    if '' not in s:
        if (True not in moving) and (False not in inpos):
            print "status:", s
            return True
    return False

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    accel = 100
    vel = 1000
    dist = 2000000
    delay = 8
    axis = sisk.X

    ser = siskiyouSerial.SiskiyouSerial(port)
    # ser.init()
    # command.setHome(sisk.X, ser)
    # command.setHome(sisk.Y, ser)
    # command.setHome(sisk.Z, ser)

    print "initial:", position(ser)

    # command.moveNegLimit(sisk.Y, ser, vel, accel)

    # command.moveRelative(sisk.X, ser, dist, vel, accel)
    # command.moveRelative(sisk.Y, ser, dist, vel, accel)
    command.moveRelative(sisk.Z, ser, dist, vel, accel)

    # while True:
    #     time.sleep(0.5)
    #     print position(ser)
    #     s = status(ser)
    #     print s
    #     if (s[0] != '' and s[1] != '' and s[2] != ''):
    #         if (s[0][15] == '0' and s[1][15] == '0' and s[2][15] == '0'):
    #             break

    # print "mid:", position(ser)

    # command.returnHome(sisk.X, ser, vel, accel)
    # command.returnHome(sisk.Y, ser, vel, accel)
    # # command.returnHome(sisk.Z, ser, vel, accel)

    while True:
        time.sleep(0.5)
        print position(ser)
        if isFinished(status(ser)):
            break
    print "limits:", (command.checkLimit(sisk.X, ser),
           command.checkLimit(sisk.Y, ser),
           command.checkLimit(sisk.Z, ser))
    print "end:", position(ser)

    ser.close()



