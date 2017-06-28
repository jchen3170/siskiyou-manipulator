#!/usr/bin/env

'''

'''

import serial

import siskiyouCommands as command
import siskiyouLibrary as sisk

def setGains(axis, P, I, D, ser):
    str1 = axis + " por " + P
    str2 = axis + " i " + I
    str3 = axis + " der " + D
    s_list = [str1, str2, str3]
    # return ser.write(B)
