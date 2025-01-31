#!/usr/bin/env python

'''
Command library to control and communicate with the manipulator

'''

import siskiyouSerial
import siskiyouLibrary as sisk

counts_per_mm = 198472 # actual conversion ratio specified by manufacturer

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

# zero all axis
# inputs:
#     ser: siskiyouSerial object
#     sp: max velocity
#     ac: acceleration rate
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
#     amt: amount to move (in encoder counts)
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
#     amt: amount to move (in encoder counts)
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

# move to hardstop in negative direction
def moveNegLimit(axis, ser, sp, ac):
    velocityMode(axis, ser, -sp, ac)

# move to hardstop in positive direction
def movePosLimit(axis, ser, sp, ac):
    velocityMode(axis, ser, sp, ac)

# start velocity mode
# inputs:
#     axis: target axis
#     ser: siskiyouSerial object
#     amt: max velocity to run at
#     ac: acceleration rate
def velocityMode(axis, ser, amt, ac):
    str1 = axis + sisk.ENABLE
    str2 = axis + sisk.ACCEL + ' ' + str(ac)
    str3 = axis + sisk.VELOCITY_MODE + ' ' + str(amt)
    s_list = [str1, str2, str3]
    ser.write_multiple(s_list)

# switch off of velocity mode back to positional control
# inputs:
#     axis: target axis
#     ser: siskiyouSerial object
#     sp: max velocity to run at
#     ac: accerleration rate
def velocityModeDisable(axis, ser, sp, ac):
    moveRelative(axis, ser, sp*12, sp, ac)

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
            print "POS_ERROR:", pos
            return ''

# get position of all axis; returns a tuple
def getPositionAll(ser):
    x = getPosition(sisk.X, ser)
    y = getPosition(sisk.Y, ser)
    z = getPosition(sisk.Z, ser)
    return (x,y,z)

# reset target axis (software power cycle)
def resetAxis(axis, ser):
    st = axis + sisk.RESET
    return ser.write(st)

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
            print "STAT_ERROR:", status
            return ''

# decodes 16-bit status response to see if moving flag is on
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

# decodes 16-bit status response to see if limit flag is on
def checkLimit(axis=None, ser=None, stat=None):
    if stat is None:
        s = getStatus(axis, ser)
    else:
        s = stat
    if len(s) < 3:
        return ''
    else:
        return s[0] == '1' or s[2] == '1'

# gets 16-bit status of all axis and checks if moving/limit
# returns a tuple of tuples
def getStatusAll(ser):
    x = getStatus(sisk.X, ser)
    y = getStatus(sisk.Y, ser)
    z = getStatus(sisk.Z, ser)
    xm = isMoving(x)
    ym = isMoving(y)
    zm = isMoving(z)
    xl = checkLimit(stat=x)
    yl = checkLimit(stat=y)
    zl = checkLimit(stat=z)

    return ((xm, ym, zm), (xl, yl, zl), (x, y, z))

# decodes 16-bit status to see if in position flag is on (finished trajectory)
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

# uses a combination of moving and inposition to determine if path is complete
def isPathComplete(ser):
    s = getStatusAll(ser)
    moving = (isMoving(s[2][0]),
              isMoving(s[2][1]),
              isMoving(s[2][2]))
    inpos = (isInPosition(s[2][0]),
             isInPosition(s[2][1]),
             isInPosition(s[2][2]))
    if ('' not in moving) and ('' not in inpos):
        if (True not in moving) and (False not in inpos):
            # print "path complete", moving, inpos
            return True
    return False

# # initializes PID gain values
# def init_controls(ser, P, I, D):
#     P = 8000 # must be between 4000 - 32000
#     I = 250 # must be between 1 - 32000
#     D = 1000 # must be between 1000 - 32000

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

# converts encoder counts to mm
def encoder2mm(val):
    if val:
        return (float(val)/float(counts_per_mm))
    else:
        return val

# converts mm to encoder counts
def mm2encoder(val):
    if val:
        return int((float(val)*float(counts_per_mm)))
    else:
        return val