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

    # read from port
    #  if called without an input, it will just read until an endline is found
    #  if called with an an input it will stop after reading that many bits
    def read(self, bits=None):
        if bits is None:
            s = self.ser.read(1024)
            # regular expression to read until an endline is found
            if s != '':
                st = re.search(r"\\r\\n.*\\r\\n", repr(s))
                # cleanup expression if a match is found
                if st:
                    return st.group().replace("\\r\\n", '')
            return s
        else:
            return self.ser.read(bits)

    # writes input command
    def write(self, s):
        # add \r\n to indicate end of command line
        s += "\r\n"
        out = self.ser.write(s)
        # read the bits written (controller repeats input command back)
        self.read(out)
        return out

    # writes multiple commands at once (ASSUMES NO OUTPUT BACK)
    def write_multiple(self, s_list):
        bits = 0
        for s in s_list:
            bits += self.write(s)

    # closes serial connection
    def close(self):
        print ("Closing serial connection...")
        self.ser.close()

    # flush outputs by continuously reading for 1 sec
    def flush(self):
        t_end = time.time() + 1
        while time.time() < t_end:
            self.ser.read()

    # infinite read from port; for debugging purposes
    def inf_read(self):
        while True:
            print repr(self.ser.readline())
            time.sleep(0.01)