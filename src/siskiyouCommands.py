#!/usr/bin/env python

'''
Command library to control and communicate with the manipulator
'''
import siskiyouSerial
import time
import siskiyouLibrary as sisk
import siskiyouControls as controls

# Sets home for selected axis for manipulator.
# inputs: 
#     axis: target axis 
#     ser: siskyouSerial object
# returns:
#     number of bytes written
def setHome(axis, ser, val=None):
    if val is None:
        st = axis + sisk.HOME
    else:
        st = axis + sisk.HOME + ' ' + str(val)
    return ser.write(st)

def zeroAll(ser, sp, ac):
    moveNegLimit(sisk.X, ser, sp, ac)
    moveNegLimit(sisk.Y, ser, sp, ac)
    moveNegLimit(sisk.Z, ser, sp, ac)

# returns selected axis to home
# inputs: 
#     axis: target axis
#     ser: siskyouSerial object
#     sp: max velocity
#     ac: acceleration rate
def returnHome(axis, ser, sp, ac):
    str1 = axis + sisk.ENABLE # enable drive
    str2 = axis + sisk.ABS + " 00000" # load absolute target position
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp) # load max commanded velocity
    str4 = axis + sisk.ACCEL + ' ' + str(ac) # load profile acceleration
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

# moves selected axis to absolute position (relative to home)
# inputs: 
#     axis: target axis
#     ser: siskyouSerial object
#     amt: amount to move
#     sp: max velocity
#     ac: acceleration rate
def moveAbsolute(axis, ser, amt, sp, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.ABS + ' ' + str(amt) 
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp)
    str4 = axis + sisk.ACCEL + ' ' + str(ac)
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

# moves selected axis to position relative to current position
# inputs: 
#     axis: target axis
#     ser: siskyouSerial object
#     amt: amount to move
#     sp: max velocity
#     ac: acceleration rate
def moveRelative(axis, ser, amt, sp, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.REL + ' ' + str(amt)
    str3 = axis + sisk.MAX_VELOCITY + ' ' + str(sp)
    str4 = axis + sisk.ACCEL + ' ' + str(ac)
    str5 = axis + sisk.MOVE
    s_list = [str1, str2, str3, str4, str5]
    # write commands to device
    ser.write_multiple(s_list)

def moveNegLimit(axis, ser, sp, ac):
    velocityMode(axis, ser, -sp, ac)

def movePosLimit(axis, ser, sp, ac):
    velocityMode(axis, ser, sp, ac)

def checkLimit(axis, ser):
    s = getStatus(axis, ser)
    while s == '':
        s = getStatus(axis, ser)
    return s[0] == '1' or s[2] == '1'

def velocityMode(axis, ser, amt, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.ACCEL + ' ' + str(ac)
    str3 = axis + sisk.VELOCITY_MODE + ' ' + str(amt)
    s_list = [str1, str2, str3]
    ser.write_multiple(s_list)

def velocityModeDisable(axis, ser):
    ac = 100
    sp = 2000
    moveRelative(axis, ser, 0, sp, ac)

# gets position of selected axis
# inputs:
#     axis: target axis
#     ser: siskyouSerial object
# outputs:
#     position of axis
def getPosition(axis, ser):
    st = axis + sisk.POSITION
    ser.write(st)
    r = ser.read()
    pos = r.split(' ')
    if len(pos) != 2:
        return ''
    elif pos[1] == '':
        return ''
    else:
        try:
            return hex2int(pos[1])
        except:
            print "ERROR:", pos
            return ''

# gets status of selected axis
# inputs:
#     axis: target axis
#     ser: siskyouSerial object
# outputs:
#     status of axis
def getStatus(axis, ser):
    st = axis + sisk.STATUS
    ser.write(st)
    r = ser.read()
    status = r.split(' ')
    if len(status) != 2:
        return ''
    elif status[1] == '':
        return ''
    else:
        try:
            return hex2bin(status[1])
        except:
            print "ERROR:", status
            return ''

def isMoving(status):
    if status == '':
        return ''
    elif len(status) != 16:
        return ''
    else:
        if status[15] == '1':
            return True
        else:
            return False

def isInPosition(status):
    if status == '':
        return ''
    elif len(status) != 16:
        return ''
    else:
        if status[14] == '1':
            return True
        else:
            return False

def init_controls(ser, P, I, D):
    P = 8000 # must be between 4000 - 32000
    I = 250 # must be between 1 - 32000
    D = 1000 # must be between 1000 - 32000
    controls.setGains(axis.ALL, P, I, D, ser)

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

def inch2encoder(val):
    counts_per_inch = 5004181
    return val*counts_per_inch