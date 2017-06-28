#!/usr/bin/env python

'''
Main script for interfacing/communicating with the Siskiyou Design 
Micromanipulator (Series MX7000). 
Assumes R232->USB and 25pin->USB

'''

import time
import serial

nl = "\r\n"

def timing(f):
    def wrap(*args):
        time1 = time.time()
        ret = f(*args)
        time2 = time.time()
        print '%s function took %0.3f ms' % (f.func_name, (time2-time1)*1000.0)
        return ret
    return wrap
    
@timing
def readSerial(ser):
    r_out = ""
    while True:
        r = ser.read()
        if (r == ""):
            break
        else:
            r_out += r
    return r_out

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    baudrate = 38400
    ser = serial.Serial(port)
    ser.timeout = 0.5
    ser.baudrate = baudrate
    print (ser)
    print "Serial Port initialized..."
    time.sleep(0.1)

    text = "2 pos" + nl

    w_out = ser.write(text)
    readSerial(ser)
    ser.flush()
    ser.write(text)

    r_out = readSerial(ser)

    print "raw:", repr(r_out)
    ret = r_out.replace(text, '').replace('\r', '').replace('\n','')
    print ret
    ser.close()
