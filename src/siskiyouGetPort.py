#!/usr/bin/env python

'''
Uses regular expression to search for the RS232 -> USB converter and output
the port address

'''

import re
import subprocess

def getPort():
    dmesg = subprocess.Popen('dmesg', stdout=subprocess.PIPE)
    dmesg = dmesg.stdout.read()

    lines = []
    port = ''
    for line in dmesg.split('\n'):
        search = re.search(r".*pl2303.*", line)
        if search:
            lines.append(search.group())
    if not lines:
        print "Device cannot be found"
        return ''

    for line in lines:
        search = re.search(r".*converter now attached to", line)
        if search:
            port = line.replace(search.group(), '').replace(' ', '')
            break
    if not lines:
        print "Device cannot be found"
        return ''

    return port

if __name__ == "__main__":
    port = getPort()
    print port