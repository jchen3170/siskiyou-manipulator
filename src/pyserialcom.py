#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Thomas Maurice"
__copyright__ = "Copyright 2013, Thomas Maurice"
__license__ = "WTFPL"
__version__ = "0.2"
__maintainer__ = "Thomas Maurice"
__email__ = "tmaurice59@gmail.com"
__status__ = "Development"

import pygtk
pygtk.require('2.0')
import gtk
import gobject
import math
import time
import pango
import serial
import os
import sys
import select
import threading
import string
import docopt

help="""pyserialcom.py - A neat serial tool in only one file !

Usage:
    pyserialcom.py [--baudrate=<BAUD>] [--port=<PORT>]

Options:
    -h --help                      Displays help
    -p PORT --port=<PORT>          The serial port file to use
    -b BAUD --baudrate=<BAUD>      The baudrate to use [default: 9600]

If no options are provided, the default 9600 baudrate will be used
and the serial port will be detected from any /dev/ttyACM* or /dev/ttyUSB*
"""

class MainWindow:
    def __init__(self, port=None, baudrate=9600):
        """
            Initialization of the software's main window
        """
        self.init_window()
        self.init_layouts()
        self.init_mode()
        self.init_sendBar()
        self.init_textview()

        if port == None:
            dirList=os.listdir("/dev/")
            for fname in dirList:
                if fname.find("ttyACM") != -1 or fname.find("ttyUSB") != -1:
                    port = "/dev/" + fname
                    print "Autodetected", port, "as a serial port"
                    break

        self.serialport = serial.Serial(port)
        self.serialport.timeout = 0
        self.serialport.baudrate = baudrate

        print self.serialport

        # some constants
        self.HEXMODE = 0
        self.ASCIIMODE = 1
        self.BOTHMODE = 2

        self.hexcount = 0
        self.mode = self.ASCIIMODE
        self.asciiButton.set_active(True)

        # Main window signals
        self.window.connect("destroy", self.exitProgram)

        # Show all
        self.mainLayout.pack_start(self.scrolledWindow, gtk.FILL|gtk.EXPAND)
        self.mainLayout.pack_start(self.sendBarLayout, gtk.SHRINK)
        self.mainLayout.pack_start(self.modeLayout, gtk.SHRINK)
        self.window.add(self.mainLayout)
        self.window.show_all()

        self.window.set_focus(self.textEntry)

        gobject.timeout_add(100, self.recieveSerial)
        gobject.timeout_add(500, self.refreshInterface)

        self.textBuffer.set_text(">>                     "+port+"@"+str(baudrate)+"bauds\n")

    def textViewChanged(self, widget, event, data=None):
        """
            This will automatically scroll the textview to the end
        """
        adj = self.scrolledWindow.get_vadjustment()
        adj.set_value(adj.upper - adj.page_size)

    def refreshInterface(self):
        """
            Will refresh the interface, setting unactivated/activated
            everything that must be
        """

    def modeChanged(self, data=None):
        if self.hexButton.get_active() == True:
            self.mode = self.HEXMODE
        elif self.asciiButton.get_active() == True:
            self.mode = self.ASCIIMODE
        else:
            self.mode = self.BOTHMODE

    def clearBuffer(self, data=None):
        self.textBuffer.set_text("")
        self.hexcount = 0

    def init_textview(self):
        """
            Initializes the text view
        """
        self.textView = gtk.TextView()
        self.textView.set_editable(False)
        fontdesc = pango.FontDescription("monospace 10")
        self.textView.modify_font(fontdesc)
        self.textBuffer = self.textView.get_buffer()
        self.scrolledWindow = gtk.ScrolledWindow()
        self.scrolledWindow.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_ALWAYS)
        self.scrolledWindow.add_with_viewport(self.textView)
        self.scrolledWindow.set_size_request(-1, 300)
        self.textView.connect('size-allocate', self.textViewChanged)

    def init_layouts(self):
        """
            Layouts !
        """
        self.modeLayout = gtk.HBox()
        self.mainLayout = gtk.VBox()
        self.sendBarLayout = gtk.HBox()

    def init_mode(self):
        self.hexButton = gtk.RadioButton(label="Hex display")
        self.hexButton.set_active(False)
        self.asciiButton = gtk.RadioButton(label="ASCII display", group=self.hexButton)
        self.asciiButton.set_active(True)
        self.bothButton = gtk.RadioButton(label="Both (ASCII+Hex)", group=self.hexButton)
        self.bothButton.set_active(True)
        self.modeLayout.pack_start(self.hexButton, gtk.SHRINK)
        self.modeLayout.pack_start(self.asciiButton, gtk.SHRINK)
        self.modeLayout.pack_start(self.bothButton, gtk.SHRINK)
        self.hexButton.connect("clicked", self.modeChanged)
        self.asciiButton.connect("clicked", self.modeChanged)
        self.bothButton.connect("clicked", self.modeChanged)

    def init_window(self):
        """
            Initialization of the main window
        """
        self.window = gtk.Window()
        self.window.set_position(gtk.WIN_POS_CENTER)
        self.window.set_default_size(480, 100)

    def init_sendBar(self):
        """
            Initializes the send bar
        """
        self.textEntry = gtk.Entry()
        self.textEntry.set_can_focus(True)
        self.sendButton = gtk.Button("Send")
        self.clearButton = gtk.Button("Clear")
        self.addCRCheckbutton = gtk.CheckButton('Add CR')
        self.addCRCheckbutton.set_active(True)

        self.textEntry.connect("activate", self.serialSend)
        self.sendButton.connect("clicked", self.serialSend)
        self.clearButton.connect("clicked", self.clearBuffer)

        self.sendBarLayout.pack_start(self.textEntry, gtk.SHRINK)
        self.sendBarLayout.pack_start(self.addCRCheckbutton, gtk.SHRINK)
        self.sendBarLayout.pack_start(self.sendButton, gtk.SHRINK)
        self.sendBarLayout.pack_start(self.clearButton, gtk.SHRINK)

    def recieveSerial(self, data=None):
        s = self.serialport.read(1024)
        if s != '':
            if self.mode == self.HEXMODE or self.mode == self.BOTHMODE:
                for c in s:
                    h = '['+hex(ord(c))
                    if len(h) < 5:
                        h += " "
                    if self.mode == self.BOTHMODE:
                        h += " "
                        if self.isCharPrintable(c) == True:
                            h += c
                        else:
                            h += "."
                    h += "]"
                    self.textBuffer.insert(self.textBuffer.get_end_iter(), h)
                    self.hexcount+=1
                    if self.hexcount >= 7:
                        self.textBuffer.insert(self.textBuffer.get_end_iter(), "\n")
                        self.hexcount = 0
            else:
                for c in s:
                    if self.isCharPrintable(c) == True or c == '\n':
                        self.textBuffer.insert(self.textBuffer.get_end_iter(), c)
                    elif c == '\r':
                        pass
                    else:
                        self.textBuffer.insert(self.textBuffer.get_end_iter(), '.')
        return True

    def isCharPrintable(self, c):
        if    (c in string.letters) or (c in string.digits) or (c in (string.punctuation+" ")):
            return True
        else:
            return False

    def serialSend(self, data=None):
        """
            Send data to the connected port
        """
        if self.textEntry.get_text() == "":
            return
            
        text = self.textEntry.get_text()
        if self.addCRCheckbutton.get_active():
            text += "\r\n"
        print ">", text
        if self.serialport.write(text) < 0:
            print "Error sending message :("
            self.textBuffer.insert(self.textBuffer.get_end_iter(), "\n>> Error sending message\n")

        self.textEntry.set_text("")

    def exitProgram(self, data=None):
        self.window.connect('event-after', gtk.main_quit)
        exit()

    def main(self):
        """
            Sofwtware's main loop
        """
        gtk.main()

if __name__ == "__main__":
    arguments = docopt.docopt(help)
    print arguments
    w = MainWindow(arguments['--port'], int(arguments['--baudrate']))
    try:
        w.main()
    except KeyboardInterrupt:
        w.window.destroy()
        print "Exited by user"
