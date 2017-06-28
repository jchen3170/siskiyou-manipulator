#!/usr/bin/env python

'''
Wrapper class for pyserial with additional functionality added.
Interfaces with and deals with all serial related code
'''

import time
import serial

class SiskiyouSerial():

    # open serial connection on initialization of class
    def __init__(self, port): 
        self.ser = serial.Serial(port)
        self.ser.baudrate = 38400
        self.ser.timeout = 0.01
        if not self.ser.isOpen():
            self.ser.open()
        print (self.ser)

    # check to make sure port is functional
    def init(self):
        print ("Initializing...")
        self.write("2 st")
        self.read()
        print ("\tComplete")

    # read from port
    def read(self, bits=None):
        if bits is None:
            r = self.ser.read(2)
            if (r == "\r\n"):
                out = ''
            else:
                out = r
            while True:
                r = self.ser.read()
                out += r
                if (r == '\n'):
                    break
                elif (r == ''):
                    print "READ TIMEOUT"
                    break
            self.ser.flush()
            return out.replace("\r\n", '')
        else:
            self.ser.flush()
            return self.ser.read(bits)

    # writes input command
    def write(self, s):
        s += "\r\n"
        out = self.ser.write(s)
        self.ser.flush()
        self.read(out)
        return out

    # writes multiple commands at once (ASSUMES NO OUTPUT BACK)
    def write_multiple(self, s_list):
        bits = 0
        for s in s_list:
            bits += self.write(s)
        # flush out all output
        # print "w_mult:", self.read(bits)

    # fixed wait time after writing for reading
    def wait(self):
        # time.sleep(0.025) # seconds
        return 0

    # closes serial connection
    def close(self):
        print ("Closing serial connection...")
        self.ser.close()

    # infinite read from port; for debugging purposes
    def inf_read(self):
        while True:
            print repr(self.ser.read())
            time.sleep(0.01)