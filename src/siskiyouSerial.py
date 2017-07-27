#!/usr/bin/env python

'''
Wrapper class for pyserial with additional functionality added.
Interfaces with and deals with all serial related code
'''

import time
import serial
import re

class SiskiyouSerial():

    # open serial connection on initialization of class
    def __init__(self, port): 
        self.ser = serial.Serial(port)
        self.ser.baudrate = 38400
        self.ser.timeout = 0.01
        if not self.ser.isOpen():
            self.ser.open()
        print (self.ser)

    # # check to make sure port is functional
    # def init(self):
    #     print ("Initializing...")
    #     self.write("2 st")
    #     self.read()
    #     print ("\tComplete")

    # read from port
    def read(self, bits=None):
        if bits is None:
            s = self.ser.read(1024)
            if s != '':
                st = re.search(r"\\r\\n.*\\r\\n", repr(s))
                if st:
                    return st.group().replace("\\r\\n", '')
            return s
        else:
            return self.ser.read(bits)

    # writes input command
    def write(self, s):
        s += "\r\n"
        out = self.ser.write(s)
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
        # time.sleep(0.02) # seconds
        return 0

    # closes serial connection
    def close(self):
        print ("Closing serial connection...")
        self.ser.close()

    def flush(self):
        t_end = time.time() + 2
        while time.time() < t_end:
            self.ser.read()

    # infinite read from port; for debugging purposes
    def inf_read(self):
        while True:
            print repr(self.ser.readline())
            time.sleep(0.01)