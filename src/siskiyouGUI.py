#!/usr/bin/env python

import Tkinter as tk
import siskiyouCommands as com
import siskiyouLibrary as sisk
import time
import Image
import ImageTk
import numpy as np

SP = 1000 # max velocity value (try not to go above 4000)
AC = 25 # acceleration rate

class Window:
    # preset program that can be run using the "PROGRAM" button in the GUI
    def setProgram(self):
        # current program moves end effector in a 5x5mm square trajectory
        program = [

        lambda:com.moveRelative(sisk.Y, self.ser, com.mm2encoder(5), SP, AC),
        lambda:com.moveRelative(sisk.X, self.ser, com.mm2encoder(5), SP, AC),
        lambda:com.moveRelative(sisk.Y, self.ser, com.mm2encoder(-5), SP, AC),
        lambda:com.moveRelative(sisk.X, self.ser, com.mm2encoder(-5), SP, AC)
        
        ]
        # save program
        self.program = program

    # manually update GUI
    def update(self):
        # udpate text/display variables
        self.pos_var.set(str(self.pos))
        self.move_var.set(str(self.moving))
        self.lims_var.set(str(self.lims))
        self.stat_var.set(str(self.raw_status))
        self.image_label.configure(image=self.image)
        # update GUI display
        self.root.update()
        # shutdown GUI if close button was hit
        if self.stop_flag:
            return True
        # handle all preset movements (calibration/program move)
        else:
            if (self.reset_flag1 or self.reset_flag2 or self.prog_flag1
                or self.prog_flag2):
                complete = com.isPathComplete(self.ser)
                # check if in calibration mode
                if self.reset_flag1:
                    if complete:
                        self.reset_p1()
                        self.reset_flag1 = False
                        self.reset_flag2 = True
                elif self.reset_flag2:
                    if complete:
                        self.reset_p2()
                        self.reset_flag2 = False
                # check if in preset program mode
                elif self.prog_flag1:
                    if complete:
                        self.prog_flag1 = False
                        self.prog_flag2 = True
                        if self.index == len(self.program):
                            self.prog_flag1 = False
                            self.prog_flag2 = False
                            self.index = 0
                        else:
                            self.nextProgram()
                            self.index += 1
                elif self.prog_flag2:
                    if complete:
                        self.prog_flag1 = True
                        self.prog_flag2 = False
                        if self.index == len(self.program):
                            self.prog_flag1 = False
                            self.prog_flag2 = False
                            self.index = 0
                        else:
                            self.nextProgram()
                            self.index += 1
            return False

    # initialize GUI components on class creation
    def __init__(self, ser):
        # specify window size
        root = tk.Tk()
        root.title("Manipulator Status")
        root.resizable(width=False, height=False)
        root.geometry("{}x{}".format(1300,700))

        font = ("TkDefaultFont",12)
        font2 = ("TkDefaultFont",12)
        font3 = ("tkDefaultFont", 7)

        # high level parent frames
        root_left = tk.Frame(root, width=524)
        root_right = tk.Frame(root)
        root_bot = tk.Frame(root)
        root_bot.pack(side="bottom", fill='x', pady=(0,5))
        root_left.pack(side="left", fill='y')
        root_right.pack(fill='x')

        # high level subframes
        root_left_sub1 = tk.Frame(root_left, relief="groove", bd=3)
        root_left_sub2 = tk.Frame(root_left, relief="groove", bd=3)
        root_left_sub3 = tk.Frame(root_left, relief="groove", bd=3)
        root_left_sub1.pack(fill="x", pady=(5,0), padx=(5,0))
        root_left_sub2.pack(fill="x", pady=(5,0), padx=(5,0))
        root_left_sub3.pack(fill="x", pady=(5,0), padx=(5,0))

        root_right_sub1 = tk.Frame(root_right)
        root_right_sub1.pack(fill="x")
        root_right_sub2 = tk.Frame(root_right, relief="groove", bd=3)
        root_right_sub2.pack(fill="x", padx=10)

        # labels for high level subframes
        values_text = tk.Label(root_left_sub1, text="Values", font=font2)
        move1_text = tk.Label(root_left_sub2, text="Basic Controls", font=font2)
        move2_text = tk.Label(root_left_sub2, text="Fixed Move", font=font2)
        adv_text = tk.Label(root_left_sub3, text="Advanced", font=font2)

        # container frames in the subframes
        container_values = tk.Frame(root_left_sub1)
        container_buttons = tk.Frame(root_left_sub2)
        container_move = tk.Frame(container_buttons)
        container_move2 = tk.Frame(container_move)
        container_adv = tk.Frame(root_left_sub3)
        container_fix_move = tk.Frame(root_left_sub2)
        image_frame = tk.Frame(root_right_sub1)
        image_frame_buttons = tk.Frame(root_right_sub2)
        image_frame_buttons2 = tk.Frame(root_right_sub2)
        values_text.pack(pady=(0,3))
        container_values.pack(anchor='w', fill="x", padx=(5,0))
        move1_text.pack(pady=(0,10))
        container_buttons.pack()
        container_move2.pack(side="left")
        move2_text.pack(pady=(10,10))
        container_fix_move.pack(pady=(0,20))
        adv_text.pack(pady=(0,10))
        container_adv.pack(pady=(0,10))
        image_frame.pack()
        image_frame_buttons.pack(pady=10)
        image_frame_buttons2.pack(pady=10)

        # subframes for the subframes
        spacing_y = 5
        frame_text = tk.Frame(container_values)
        frame_value = tk.Frame(container_values)
        frame_buttons_top = tk.Frame(container_buttons)
        frame_buttons_mid = tk.Frame(container_move2)
        frame_buttons_bot = tk.Frame(container_buttons)
        frame_buttons_mid2 = tk.Frame(container_move2)
        frame_entry_x = tk.Frame(container_fix_move)
        frame_entry_y = tk.Frame(container_fix_move)
        frame_entry_z = tk.Frame(container_fix_move)
        entry_units_x = tk.Frame(container_fix_move, width=25)
        entry_units_y = tk.Frame(container_fix_move, width=25)
        entry_units_z = tk.Frame(container_fix_move, width=25)
        frame_text.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_value.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_buttons_top.pack(anchor="w", pady=spacing_y)
        frame_buttons_mid.pack(side="top", anchor="w", pady=spacing_y)
        frame_buttons_mid2.pack(side="bottom", anchor="w", pady=spacing_y)
        container_move.pack(anchor="w")
        frame_buttons_bot.pack(anchor="w", pady=spacing_y)
        frame_entry_x.pack(side="left")
        entry_units_x.pack(side="left", fill='y')
        frame_entry_y.pack(side="left", fill='y')
        entry_units_y.pack(side="left", fill='y')
        frame_entry_z.pack(side="left", fill='y')
        entry_units_z.pack(side="left", fill='y')

        # text labels for the status frame
        text = tk.Label(frame_text, text= "", font=font)
        text1 = tk.Label(frame_text, text="Position: ", font=font)
        text2 = tk.Label(frame_text, text="Moving: ", font=font)
        text3 = tk.Label(frame_text, text="Limits: ", font=font)
        text4 = tk.Label(frame_text, text="Status: ", font=font)
        text.pack(anchor="w", pady=5)
        text1.pack(anchor="w", pady=5)
        text2.pack(anchor="w", pady=5)
        text3.pack(anchor="w", pady=5)
        text4.pack(anchor="w", pady=5)

        # changing variables that show current status
        self.pos_var = tk.StringVar()
        self.move_var = tk.StringVar()
        self.lims_var = tk.StringVar()
        self.stat_var = tk.StringVar()
        self.vel_var = tk.StringVar()
        self.pos = (0, 0, 0)
        self.pos_var.set(str(self.pos))
        self.moving = ('', '', '')
        self.move_var.set(str(self.moving))
        self.lims = ('', '', '')
        self.lims_var.set(str(self.lims))
        self.raw_status = ('', '', '')
        self.stat_var.set(str(self.raw_status))
        self.vel = (0, 0, 0)
        self.vel_var.set(str(self.vel))

        # create labels to display the changing statuses
        xyz = tk.Label(frame_value, text="(X, Y, Z)", font=font)
        position = tk.Label(frame_value, textvariable=self.pos_var, font=font)
        moving = tk.Label(frame_value, textvariable=self.move_var, font=font)
        limits = tk.Label(frame_value, textvariable=self.lims_var, font=font)
        status = tk.Label(frame_value, textvariable=self.stat_var, 
            font=("TkDefaultFont",9))
        xyz.pack(anchor="w", pady=5)
        position.pack(anchor="w", pady=5)
        moving.pack(anchor="w", pady=5)
        limits.pack(anchor="w", pady=5)
        status.pack(anchor="w", pady=5)

        # buttons to trigger the zero commands
        pad_x_button = 15
        zero_x = tk.Button(frame_buttons_top, text="Zero X", takefocus=False,
            command= lambda: self.zero(sisk.X))      
        zero_y = tk.Button(frame_buttons_top, text="Zero Y", takefocus=False,
            command= lambda: self.zero(sisk.Y))       
        zero_z = tk.Button(frame_buttons_top, text="Zero Z", takefocus=False,
            command= lambda: self.zero(sisk.Z))        
        zero_all = tk.Button(frame_buttons_top,text="Zero ALL", takefocus=False,
            command= lambda: self.zeroAll())
        zero_x.pack(side="left", padx=(10,pad_x_button))
        zero_y.pack(side="left", padx=pad_x_button)
        zero_z.pack(side="left", padx=pad_x_button)
        zero_all.pack(side="left", padx=pad_x_button)

        # buttons to trigger continuous movement in positive direction
        aa = 6
        move_x = tk.Button(frame_buttons_mid, text="Move +X", takefocus=False,
            command= lambda: self.move(sisk.X, True))
        move_y = tk.Button(frame_buttons_mid, text="Move +Y", takefocus=False,
            command= lambda: self.move(sisk.Y, True))
        move_z = tk.Button(frame_buttons_mid, text="Move +Z", takefocus=False,
            command= lambda: self.move(sisk.Z, True))
        home = tk.Button(container_move, text="Home", takefocus=False,
            command=self.returnHome)
        move_x.pack(side="left", padx=(3,pad_x_button-aa))
        move_y.pack(side="left", padx=pad_x_button-aa)
        move_z.pack(side="left", padx=pad_x_button-aa)
        home.pack(side='right', padx=20)

        # buttons to trigger continuous movement in negative direction
        aa = 5
        move_xn = tk.Button(frame_buttons_mid2,text="Move  -X", takefocus=False,
            command= lambda: self.move(sisk.X, False))
        move_yn = tk.Button(frame_buttons_mid2,text="Move  -Y", takefocus=False,
            command= lambda: self.move(sisk.Y, False))
        move_zn = tk.Button(frame_buttons_mid2,text="Move  -Z", takefocus=False,
            command= lambda: self.move(sisk.Z, False))
        move_xn.pack(side="left", padx=(3,pad_x_button-aa))
        move_yn.pack(side="left", padx=pad_x_button-aa)
        move_zn.pack(side="left", padx=pad_x_button-aa)

        # buttons to stop movement in specific/all axis
        stop_x = tk.Button(frame_buttons_bot, text="Stop X", takefocus=False,
            command= lambda: self.stopMove(sisk.X))
        stop_y = tk.Button(frame_buttons_bot, text="Stop Y", takefocus=False,
            command= lambda: self.stopMove(sisk.Y))
        stop_z = tk.Button(frame_buttons_bot, text="Stop Z", takefocus=False,
            command= lambda: self.stopMove(sisk.Z))
        stop_all = tk.Button(frame_buttons_bot, text="Stop ALL",takefocus=False,
            command= lambda: self.stopAll())
        stop_x.pack(side="left", padx=(10,pad_x_button))
        stop_y.pack(side="left", padx=pad_x_button)
        stop_z.pack(side="left", padx=pad_x_button)
        stop_all.pack(side="left", padx=pad_x_button)

        # buttons for advanced controls
        prog = tk.Button(container_adv, text="PROGRAM", font=font3, 
            command=self.programMove, takefocus=False)
        calib = tk.Button(container_adv, text="CALIBRATE", font=font3,
            command=self.calibrate, takefocus=False)
        flush = tk.Button(container_adv, text="FLUSH", font=font3,
            command=self.flush, takefocus=False)
        power_cycle = tk.Button(container_adv, text="POWER CYCLE", font=font3,
            command=self.pcycle, takefocus=False)
        prog.pack(side="left")
        calib.pack(side="left", padx=(25,25))
        flush.pack(side="left", padx=(25,25))
        power_cycle.pack(side="left")

        # buttons/entries for fixed distance movement
        vcmd = (root.register(self.entryValid),
                '%d', '%i', '%P', '%s', '%S')
        entry_x = tk.StringVar()
        entry_y = tk.StringVar()
        entry_z = tk.StringVar()
        entry_x = tk.Entry(frame_entry_x, textvariable=entry_x, width=12,
            font=font, validate='key', vcmd=vcmd, bd=2, relief="ridge")
        entry_y = tk.Entry(frame_entry_y, textvariable=entry_y, width=12,
            font=font, validate='key', vcmd=vcmd, bd=2, relief="ridge")
        entry_z = tk.Entry(frame_entry_z, textvariable=entry_z, width=12,
            font=font, validate='key', vcmd=vcmd, bd=2, relief="ridge")
        entry_x.pack()
        entry_y.pack()
        entry_z.pack()
        self.entry_x = entry_x
        self.entry_y = entry_y
        self.entry_z = entry_z
        entry_x_button = tk.Button(frame_entry_x, text="Move X",takefocus=False,
            command = lambda: self.fixedMove(sisk.X,self.entry_x))
        entry_y_button = tk.Button(frame_entry_y, text="Move Y",takefocus=False,
            command = lambda: self.fixedMove(sisk.Y,self.entry_y))
        entry_z_button = tk.Button(frame_entry_z, text="Move Z",takefocus=False,
            command = lambda: self.fixedMove(sisk.Z,self.entry_z))
        entry_x_button.pack(pady=(8,0))
        entry_y_button.pack(pady=(8,0))
        entry_z_button.pack(pady=(8,0))

        xyz_units = tk.StringVar()
        xyz_units.set('cts')
        self.xyz_units = xyz_units
        x_mm_cts = tk.Label(entry_units_x, textvariable=xyz_units, font=font3)
        y_mm_cts = tk.Label(entry_units_y, textvariable=xyz_units, font=font3)
        z_mm_cts = tk.Label(entry_units_z, textvariable=xyz_units, font=font3)
        x_mm_cts.pack(side="top", pady=6)
        y_mm_cts.pack(side="top", pady=6)
        z_mm_cts.pack(side="top", pady=6)

        self.unit_flag = False
        self.unit_label_var = tk.StringVar()
        self.unit_label_var.set("units: " + xyz_units.get())
        change_units_button = tk.Button(root_left_sub1, text="Change Units",
            command=self.unitSwap, takefocus=False)
        change_units_button.pack(pady=(0,2))
        unit_label = tk.Label(root_left_sub1, textvariable=self.unit_label_var,
            font=('TkDefaultFont',9))
        unit_label.pack(pady=(0,5))

        # image display and buttons
        img = Image.fromarray(np.ones([500,800]))
        imgTk = ImageTk.PhotoImage(img)
        self.image = imgTk
        image_label = tk.Label(image_frame, image=self.image)
        image_label.pack(anchor="e")
        image_label.bind("<ButtonPress-1>", self.imagePoint)
        image_move = tk.Button(image_frame_buttons, text="Move",
            takefocus=False, command= lambda: self.setMoveFlag(True))
        image_pt_undo = tk.Button(image_frame_buttons, text="Undo", 
            takefocus=False, command=self.undoImagePoint)
        image_pt_reset = tk.Button(image_frame_buttons, text="Reset",
            takefocus=False, command=self.resetImagePoints)
        image_move.pack(side="left", padx=25)
        image_pt_undo.pack(side="left", padx=25)
        image_pt_reset.pack(side="left", padx=25)

        image_edge = tk.Button(image_frame_buttons2, text="Show Edges",
            takefocus=False, command= self.setEdgeFlag)
        image_contour = tk.Button(image_frame_buttons2, text="Show Contour",
            takefocus=False, command= self.setContourFlag)
        image_edge.pack(side="left", padx=36)
        image_contour.pack(side="left", padx=25)

        # close button
        close_button = tk.Button(root_bot, text="Close", command=self.stop,
            takefocus=False)
        close_button.pack()

        # disable automatic resizing of some frames
        root_left.pack_propagate(0)
        entry_units_x.pack_propagate(0)
        entry_units_y.pack_propagate(0)
        entry_units_z.pack_propagate(0)

        # assign important variables to class level
        self.root = root
        self.ser = ser
        self.image_label = image_label
        self.image_points = []
        self.stop_flag = False
        self.reset_flag1 = False
        self.reset_flag2 = False
        self.move_flag = False
        self.edge_flag = False
        self.contour_flag = False
        self.prog_flag1 = False
        self.prog_flag2 = False
        self.index = 0
        self.SP = SP
        self.AC = AC
        self.setProgram()

    # zero the specified axis
    def zero(self, axis):
        com.setHome(axis, self.ser)

    # zero all axis
    def zeroAll(self):
        self.zero(sisk.X)
        self.zero(sisk.Y)
        self.zero(sisk.Z)

    # move specified axis in the specified direction
    def move(self, axis, dir):
        if not (self.reset_flag1 or self.reset_flag2 or self.prog_flag1 or 
            self.prog_flag2):
            if dir:
                com.velocityMode(axis, self.ser, SP, AC)
                self.setVelocity(axis, SP)
            else:
                com.velocityMode(axis, self.ser, -SP, AC)
                self.setVelocity(axis, -SP)
        else:
            print "Currently busy"

    # stop specified axis
    def stopMove(self, axis):
        com.velocityModeDisable(axis, self.ser, self.getVelocity(axis), AC)
        self.setVelocity(axis, 0)
        self.reset_flag1 = False
        self.reset_flag2 = False
        self.move_flag = False
        self.prog_flag1 = False
        self.prog_flag2 = False
        self.index = 0

    # stop all axis
    def stopAll(self):
        self.stopMove(sisk.X)
        self.stopMove(sisk.Y)
        self.stopMove(sisk.Z)

    # return all axis to Home position
    def returnHome(self):
        com.returnHome(sisk.X, self.ser, SP, AC)
        com.returnHome(sisk.Y, self.ser, SP, AC)
        com.returnHome(sisk.Z, self.ser, SP, AC)

    # initiate reset/calibration
    def calibrate(self):
        com.zeroAll(self.ser, SP, AC)
        self.reset_flag1 = True
        self.prog_flag1 = False
        self.prog_flag2 = False

    # flush controller output to try to fix communication problems
    def flush(self):
        self.ser.flush()

    # reset controller unit (software reset/power cycle)
    def pcycle(self):
        com.resetAxis(sisk.X, self.ser)
        com.resetAxis(sisk.Y, self.ser)
        com.resetAxis(sisk.Z, self.ser)
        self.reset_flag1 = False
        self.reset_flag2 = False
        self.move_flag = False
        self.prog_flag1 = False
        self.prog_flag2 = False
        self.index = 0

    # update internal position variable
    def setPosition(self, pos):
        if self.unit_flag:
            sigfig = 8
            x = ''
            y = ''
            z = ''
            if pos[0]:
                x = "%.8f" % round(com.encoder2mm(pos[0]),sigfig)
            elif pos[0] == 0:
                x = "%.8f" % float(0.00000000)
            if pos[1]:
                y = "%.8f" % round(com.encoder2mm(pos[1]),sigfig)
            elif pos[1] == 0:
                y = "%.8f" % float(0.00000000)
            if pos[2]:
                z = "%.8f" % round(com.encoder2mm(pos[2]),sigfig)
            elif pos[2] == 0:
                z = "%.8f" % float(0.00000000)
            self.pos = (x,y,z)
        else: 
            self.pos = pos

    # update internal moving variable
    def setMoving(self, move):
        self.moving = move

    # update internal limits variable
    def setLimits(self, lims):
        self.lims = lims

    # update internal status variable
    def setStatus(self, stat):
        self.raw_status = stat

    # built-in self update (not recommended)
    def start(self):
        self.root.mainloop()

    # stop function that indicates close button has been hit
    def stop(self):
        self.stop_flag = True
        self.reset_flag1 = False
        self.reset_flag2 = False
        self.prog_flag1 = False
        self.prog_flag2 = False
        self.index = 0

    # part 1 of calibration: move to set position after hitting limits
    def reset_p1(self):
        com.moveRelative(sisk.X, self.ser, 2000000, SP, AC)
        com.moveRelative(sisk.Y, self.ser, 2000000, SP, AC)
        com.moveRelative(sisk.Z, self.ser, 2000000, SP, AC)

    # part 2 of calibration: zero all axis
    def reset_p2(self):
        com.setHome(sisk.X, self.ser)
        com.setHome(sisk.Y, self.ser)
        com.setHome(sisk.Z, self.ser)

    # get the current set velocity of specified axis
    def getVelocity(self, axis):
        if axis is sisk.X:
            return self.vel[0]
        elif axis is sisk.Y:
            return self.vel[1]
        else:
            return self.vel[2]

    # set velocity of input axis by the axis amt value
    def setVelocity(self, axis, amt):
        if axis is sisk.X:
            self.vel = (amt, self.vel[1], self.vel[2])
        elif axis is sisk.Y:
            self.vel = (self.vel[0], amt, self.vel[2])
        else:
            self.vel = (self.vel[0], self.vel[1], amt)

    # set the image currently on display
    def setImage(self, img):
        img = Image.fromarray(img)
        img = ImageTk.PhotoImage(img)
        self.image = img

    # get and save the coordinates of the click on the image (relative to image)
    def imagePoint(self,event):
        x = event.x
        y = event.y
        self.image_points.append((x,y))

    # return all saved image click coordinates
    def getImagePoints(self):
        return self.image_points

    # undo most recent image click point
    def undoImagePoint(self):
        if self.image_points:
            del(self.image_points[-1])
            if not self.image_points:
                self.move_flag = False

    # remove all image click points
    def resetImagePoints(self):
        del self.image_points[:]
        self.move_flag = False

    # remove the first image click point
    def removeFirstImagePoint(self):
        if self.image_points:
            del self.image_points[0]
    
    # begin movement towards first image point
    def setMoveFlag(self, flag):
        self.move_flag = flag
        self.reset_flag1 = False
        self.reset_flag2 = False
        self.prog_flag1 = False
        self.prog_flag2 = False
        self.index = 0

    # change status of movement towards image point
    def getMoveFlag(self):
        return self.move_flag

    # checks to make sure entered value into entry widget is numerical
    def entryValid(self, d, i, P, s, text):
        # always allow deletes
        if d == '0':
            return True
        # make sure number value never exceeds 7 digits for cts
        if self.unit_flag:
            if len(P.replace('-','').replace('.','')) > 9:
                return False
        else:
            if len(P.replace('-','')) > 7:
                return False
        # check entered key
        if i == '0':
            # make sure there is only ever one neg. sign and period
            if s:
                if text == '-':
                    return ('-' not in s)
                elif text == '.':
                    return self.unit_flag and ('.' not in s) and ('-' not in s)
                else:
                    return text.isdigit()
            # allow for insertion of neg sign / period at beginning
            else:
                if self.unit_flag:
                    return text.isdigit() or text == '-' or text == '.'
                else:
                    return text.isdigit() or text == '-'
        else:
            # make sure there is only ever one period
            if text == '.':
                if self.unit_flag:
                    return ('.' not in s)
                else:
                    return False
            # make sure value is a digit
            else:
                return text.isdigit()

    # moves axis a fixed distance
    def fixedMove(self, axis, val):
        amt = val.get()
        # confirm that textbox entry is actually an integer
        try:
            if not self.unit_flag:
                int(amt)
            else:
                amt = com.mm2encoder(amt)
            com.moveRelative(axis, self.ser, amt, SP, AC)
            return True
        except ValueError:
            return False

    # adjusts settings for overlaying edge map
    def setEdgeFlag(self):
        self.edge_flag = not self.edge_flag

    # returns current flag state
    def getEdgeFlag(self):
        return self.edge_flag

    # adjusts settings for overlaying contour map
    def setContourFlag(self):
        self.contour_flag = not self.contour_flag

    # returns current flag state
    def getContourFlag(self):
        return self.contour_flag

    # swaps the unit listing and appropriate flags
    def unitSwap(self):
        if self.unit_flag:
            self.xyz_units.set('cts')
        else:
            self.xyz_units.set('mm')
        self.unit_flag = not self.unit_flag
        self.unit_label_var.set("units: " + self.xyz_units.get())

    # begin program move
    def programMove(self):
        # make sure there are commands in the program list
        if self.program:
            self.prog_flag1 = True
            self.nextProgram()
            self.index += 1

    # call next program in program list
    def nextProgram(self):
        f = self.program[self.index]
        f()

# test code for when executing the class directly
if __name__ == "__main__":
    aa = (1000000, 1000000, 1000000)
    cc = 1
    gui = Window()
    while True:
        # aa = (aa[0]+cc, aa[1]+cc, aa[2]+cc)
        gui.setPosition(aa)
        if gui.update():
            break
        time.sleep(0.1)