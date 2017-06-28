#!/usr/bin/env python

import re
import subprocess

def show_devices():
    device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
    df = subprocess.check_output("lsusb")
    devices = []
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                devices.append(dinfo)
    return devices

def print_devices(devices):
    for i in devices:
        print str(i) + "\n"
    
def toString(devices):
    s = ""
    for i in devices:
        s += str(i) + "\n"
    return s

if __name__ == "__main__":
    devices = show_devices()
    print_devices(devices)