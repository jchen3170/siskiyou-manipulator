#!/usr/bin/env python

'''
Motion library to control and communicate with Siskiyou Design Micromanipulator 
(MX7000 Series).
Interface is assuming RS-232 -> USB and 25pin -> USB.
'''
import siskiyouSerial
import time
import siskiyouLibrary as sisk

# Sets home for selected axis for manipulator.
# inputs: 
#     axis: target axis 
#     ser: python serial object
# returns:
#     number of bytes written
def setHome(axis, ser, val=None):
    if val is None:
        st = axis + sisk.HOME
    else:
        st = axis + sisk.HOME + ' ' + str(val)
    return ser.write(st)

# returns selected axis to home
# inputs: 
#     axis: target axis
#     sp: max velocity
#     ac: acceleration rate
#     ser: python serial object
def returnHome(axis, ser, sp, ac):
    str1 = axis + sisk.ENABLE # enable drive
    str2 = axis + sisk.ABS + " 00000" # load absolute target position
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp) # load max commanded velocity
    str4 = axis + sisk.ACCEL + ' ' + str(ac) # load profile acceleration
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

def absPosition(axis, ser, amt, sp, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.ABS + ' ' + str(amt) 
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp)
    str4 = axis + sisk.ACCEL + ' ' + str(ac)
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

def relPosition(axis, ser, amt, sp, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.REL + ' ' + str(amt)
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp)
    str4 = axis + sisk.ACCEL + ' ' +str(ac)
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

def getPosition(axis, ser):
    st = axis + sisk.POSITION
    ser.write(st)
    ser.wait()
    r = ser.read()
    pos = r.split(' ')
    if len(pos) != 2:
        return ''
    return hex2int(pos[1])

def getStatus(axis, ser):
    st = axis + sisk.STATUS
    ser.write(st)
    ser.wait()
    r = ser.read()
    status = r.split(' ')
    if len(status) != 2:
        return ''
    return hex2bin(status[1])

# convert hex string to binary
def hex2bin(st):
    scale = 16
    num_bits = 16
    return bin(int(st, scale))[2:].zfill(num_bits)

# convert hex string to SIGNED int
def hex2int(st):
    val = int(st,16)
    if val >= 0x7FFFFFFF:
        val -= 0x100000000
    return val