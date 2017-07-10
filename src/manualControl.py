#!/usr/bin/env python

'''
Manual control script for the manipulator
'''

import curses
import time

import siskiyouSerial
import siskiyouCommands as command
import siskiyouGetPort as port
import siskiyouLibrary as sisk

ser = None

def main(stdscr):
    stdscr = curses.initscr()
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(1)
    stdscr.refresh()
    stdscr.nodelay(1)

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.refresh()
        if key == curses.KEY_UP:
            # command.moveRelative(sisk.Y, ser, dist, vel, accel)
            command.velocityMode(sisk.Y, ser, vel, accel)
        elif key == curses.KEY_DOWN:
            command.velocityMode(sisk.Y, ser, -vel, accel)
        elif key == curses.KEY_LEFT:
            command.velocityMode(sisk.X, ser, -vel, accel)
        elif key == curses.KEY_RIGHT:
            command.velocityMode(sisk.X, ser, vel, accel)
        elif key == ord('s'):
            # command.moveRelative(sisk.Z, ser, dist, vel, accel)
            command.velocityMode(sisk.Z, ser, vel, accel)
        elif key == ord('w'):
            # command.moveRelative(sisk.Z, ser, -dist, vel, accel)
            command.velocityMode(sisk.Z, ser, -vel, accel)
        elif key == ord('R'):
            stdscr.clear()
            stdscr.addstr(0,0,"Resetting positions...")
            stdscr.refresh()
            pos = zero_middle(ser, dist, vel, accel, stdscr)
            stdscr.addstr(2,0,"position: " + str(pos))
        elif key == ord('X'):
            command.velocityModeDisable(sisk.X, ser)
        elif key == ord('Y'):
            command.velocityModeDisable(sisk.Y, ser)
        elif key == ord('Z'):
            command.velocityModeDisable(sisk.Z, ser)
        elif key == ord('P'):
            command.velocityModeDisable(sisk.X, ser)
            command.velocityModeDisable(sisk.Y, ser)
            command.velocityModeDisable(sisk.Z, ser)

        stdscr.clear()
        pos = position(ser)
        stat = status(ser)
        lims = limits(ser)
        stdscr.addstr(0,0,
            "'q' to quit; 'up/down' for Y; 'left/right' for X; 's/w' for Z")
        stdscr.addstr(2,0,"position: " + str(pos))
        stdscr.addstr(4,0,"status: " + str(stat))
        stdscr.addstr(6,0,"limits: " + str(lims))

    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()

def position(ser):
    x_pos = command.getPosition(sisk.X, ser)
    y_pos = command.getPosition(sisk.Y, ser)
    z_pos = command.getPosition(sisk.Z, ser)
    return (x_pos, y_pos, z_pos)

def status(ser):
    x_status = command.getStatus(sisk.X, ser)
    y_status = command.getStatus(sisk.Y, ser)
    z_status = command.getStatus(sisk.Z, ser)
    return (x_status, y_status, z_status)

def limits(ser):
    lim = (command.checkLimit(sisk.X, ser),
           command.checkLimit(sisk.Y, ser),
           command.checkLimit(sisk.Z, ser))
    return lim

def isFinished(s,stdscr=None):
    moving = (command.isMoving(s[0]),
              command.isMoving(s[1]),
              command.isMoving(s[2]))
    inpos = (command.isInPosition(s[0]),
             command.isInPosition(s[1]),
             command.isInPosition(s[2]))
    if '' not in s:
        if (True not in moving) and (False not in inpos):
            if (stdscr is None):
                print "status:", s
            else:
                stdscr.addstr(4,0,"status: " + str(s))
                stdscr.refresh()
            return True
    return False

def wait(ser, stdscr=None):
    while not isFinished(status(ser),stdscr):
        time.sleep(0.5)
        pos = position(ser)
        if (stdscr is not None):
            stdscr.clear()
            stdscr.addstr(0,0,"Resetting positions...")
            stdscr.addstr(2,0,"position: " + str(pos))
            stdscr.refresh()
        else:
            print str(pos)

    if (stdscr is not None):
        stdscr.addstr(6,0,"limits: " + str(limits))
        stdscr.refresh()
    else:
        print "limits:", limits(ser)

def zero_middle(ser, dist, vel, accel, stdscr):
    init_pos = position(ser)

    command.zeroAll(ser, vel, accel)

    wait(ser, stdscr)
    mid_pos = position(ser)

    command.moveRelative(sisk.X, ser, 2000000, vel, accel)
    command.moveRelative(sisk.Y, ser, 2000000, vel, accel)
    command.moveRelative(sisk.Z, ser, 2000000, vel, accel)

    wait(ser, stdscr)
    time.sleep(0.5)
    end_pos = position(ser)

    command.setHome(sisk.X, ser)
    command.setHome(sisk.Y, ser)
    command.setHome(sisk.Z, ser)

    return (init_pos, mid_pos, end_pos)

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    dist = 200000
    vel = 2000
    accel = 100
    ser = siskiyouSerial.SiskiyouSerial(port)

    curses.wrapper(main)
    wait(ser)