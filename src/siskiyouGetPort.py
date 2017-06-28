#!/usr/bin/env python

import re
import rospy

import show_devices as sd

device_id = "067b:2303"

def getPort():
    devices = sd.toString(sd.show_devices())
    line = re.findall("^.*"+ device_id +".*$",devices,re.MULTILINE)

    if len(line) == 0:
        raise ValueError("Device ID not found")
        return ""
    elif len(line) > 1:
        raise ValueError("Multiple device ID's found")
        return ""

    subline = re.findall("'/dev/bus/.*?'",line[0])

    if len(subline) == 0:
        raise ValueError("Address not found in device ID line")
        return ""
    elif len(subline) > 1:
        raise ValueError("Multiple matches in device ID line")

    port = re.sub("'","",subline[0])
    return port