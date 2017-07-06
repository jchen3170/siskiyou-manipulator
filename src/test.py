#!/usr/bin/env python

import time

import siskiyouSerial
import siskiyouLibrary as sisk
import siskiyouCommands as command

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    ser = siskiyouSerial.SiskiyouSerial(port)
    axis = sisk.Y

    while True:
        print command.getPosition(axis, ser)
        time.sleep(0.1)


