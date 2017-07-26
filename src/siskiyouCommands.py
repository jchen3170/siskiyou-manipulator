#!/usr/bin/env python

'''
Command library to control and communicate with the manipulator
'''
import siskiyouSerial
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

def getPositionAll(ser):
    x = getPosition(sisk.X, ser)
    y = getPosition(sisk.Y, ser)
    z = getPosition(sisk.Z, ser)
    return (x,y,z)

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

def checkLimit(axis=None, ser=None, stat=None):
    if stat is None:
        s = getStatus(axis, ser)
    else:
        s = stat
    if len(s) < 3:
        return ''
    else:
        return s[0] == '1' or s[2] == '1'

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

def isPathComplete(ser):
    s = getStatusAll(ser)
    moving = (isMoving(s[2][0]),
              isMoving(s[2][1]),
              isMoving(s[2][2]))
    inpos = (isInPosition(s[2][0]),
             isInPosition(s[2][1]),
             isInPosition(s[2][2]))
    if '' not in s:
        if (True not in moving) and (False not in inpos):
            return True
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