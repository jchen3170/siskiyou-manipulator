#!/usr/bin/env python

'''

'''

import time
import serial

class SiskiyouSerial():

    # open serial connection on initialization of class
    def __init__(self, port): 
        self.ser = serial.Serial(port)
        self.ser.baudrate = 38400
        self.ser.timeout = 0.1
        if not self.ser.isOpen():
            self.ser.open()
        print (self.ser)

    # check to make sure port is functional
    def init(self):
        print ("Initializing...")
        self.write("2 st")
        time.sleep(0.01)
        self.read()
        print ("\tComplete")

    # read from port
    def read(self, bits=None):
        if bits is None:
            n_end = 3
        else:
            a= self.ser.read(bits)
            print "read:", repr(a)
            return a
        n = 0
        r_out = ""
        while True:
            r = self.ser.read()
            # print repr(r)
            if (r == ''):
                print "Read timeout"
                break
            else:
                r_out += r
                if (r == '\n'):
                    n += 1
                    if (n >= n_end):
                        break
        print "read:", repr(r_out)
        return r_out

    # writes input command
    def write(self, s):
        print "write:", repr(s + "\r\n")
        # print s
        # time.sleep(0.05)
        return self.ser.write(s + "\r\n")

    # writes multiple commands at once (ASSUMES NO OUTPUT BACK)
    def write_multiple(self, s_list):
        bits = 0
        for s in s_list:
            bits += self.write(s)
        self.wait()
        # flush out all output
        self.read(bits)

    # fixed wait time after writing for reading
    def wait(self):
        time.sleep(0.025) # seconds

    # closes serial connection
    def close(self):
        print ("Closing serial connection...")
        self.ser.close()

    # infinite read from port; for debugging purposes
    def inf_read(self):
        while True:
            print repr(self.ser.read())
            time.sleep(0.01)