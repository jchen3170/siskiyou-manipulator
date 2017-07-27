#!/usr/bin/env python

import Tkinter as tk
import siskiyouCommands as com
import siskiyouLibrary as sisk
import time
import Image
import ImageTk
import numpy as np

sp = 500
ac = 25
class Window:
    def __init__(self, ser):
        root = tk.Tk()
        root.title("Manipulator Status")
        root.resizable(width=False, height=False)
        root.geometry("{}x{}".format(1400,700))

        font = ("TkDefaultFont",12)
        pad_y = 5

        root_left = tk.Frame(root, bg='green')
        root_right = tk.Frame(root, bg='red')
        root_bot = tk.Frame(root, bg='blue')
        root_bot.pack(side="bottom", fill='x', pady=(0,10))
        root_left.pack(side="left", fill='y')
        root_right.pack(fill='both')
        
        container = tk.Frame(root_left)
        container2 = tk.Frame(root_left)
        container_move = tk.Frame(container2)
        container_move2 = tk.Frame(container_move)
        container_adv = tk.Frame(root_left)
        image_frame = tk.Frame(root_right)
        container.pack(anchor="w")
        container2.pack(anchor="w", pady=(25,0))
        container_adv.pack(anchor="w",pady=(50,0))
        container_move2.pack(side="left")
        image_frame.pack()

        spacing_y = 5
        frame_text = tk.Frame(container)
        frame_value = tk.Frame(container)
        frame_buttons_top = tk.Frame(container2)
        frame_buttons_mid = tk.Frame(container_move2)
        frame_buttons_bot = tk.Frame(container2)
        frame_buttons_mid2 = tk.Frame(container_move2)
        frame_text.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_value.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_buttons_top.pack(anchor="w", pady=spacing_y)
        frame_buttons_mid.pack(side="top", anchor="w", pady=spacing_y)
        frame_buttons_mid2.pack(side="bottom", anchor="w", pady=spacing_y)
        container_move.pack(anchor="w")
        frame_buttons_bot.pack(anchor="w", pady=spacing_y)

        text = tk.Label(frame_text, text= "", font=font)
        text1 = tk.Label(frame_text, text="Position: ", font=font)
        text2 = tk.Label(frame_text, text="Moving: ", font=font)
        text3 = tk.Label(frame_text, text="Limits: ", font=font)
        text4 = tk.Label(frame_text, text="Status: ", font=font)
        text5 = tk.Label(frame_text, text="Velocity: ", font=font)
        text.pack(anchor="w", pady=pad_y)
        text1.pack(anchor="w", pady=pad_y)
        text2.pack(anchor="w", pady=pad_y)
        text3.pack(anchor="w", pady=pad_y)
        text4.pack(anchor="w", pady=pad_y)
        text5.pack(anchor="w", pady=pad_y)

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

        xyz = tk.Label(frame_value, text="(X, Y, Z)", font=font)
        position = tk.Label(frame_value, textvariable=self.pos_var, font=font)
        moving = tk.Label(frame_value, textvariable=self.move_var, font=font)
        limits = tk.Label(frame_value, textvariable=self.lims_var, font=font)
        status = tk.Label(frame_value, textvariable=self.stat_var, 
            font=("TkDefaultFont",10))
        velocity = tk.Label(frame_value, textvariable=self.vel_var, font=font)
        xyz.pack(anchor="w", pady=pad_y)
        position.pack(anchor="w", pady=pad_y)
        moving.pack(anchor="w", pady=pad_y)
        limits.pack(anchor="w", pady=pad_y)
        status.pack(anchor="w", pady=pad_y)
        velocity.pack(anchor="w", pady=pad_y)

        pad_x_button = 15
        zero_x = tk.Button(frame_buttons_top, text="Zero X", 
            command= lambda: self.zero(sisk.X))      
        zero_y = tk.Button(frame_buttons_top, text="Zero Y", 
            command= lambda: self.zero(sisk.Y))       
        zero_z = tk.Button(frame_buttons_top, text="Zero Z", 
            command= lambda: self.zero(sisk.Z))        
        zero_all = tk.Button(frame_buttons_top, text="Zero ALL",
            command= lambda: self.zeroAll())
        zero_x.pack(side="left", padx=(10,pad_x_button))
        zero_y.pack(side="left", padx=pad_x_button)
        zero_z.pack(side="left", padx=pad_x_button)
        zero_all.pack(side="left", padx=pad_x_button)

        aa = 6
        move_x = tk.Button(frame_buttons_mid, text="Move +X", 
            command= lambda: self.move(sisk.X, True))
        move_y = tk.Button(frame_buttons_mid, text="Move +Y", 
            command= lambda: self.move(sisk.Y, True))
        move_z = tk.Button(frame_buttons_mid, text="Move +Z", 
            command= lambda: self.move(sisk.Z, True))
        home = tk.Button(container_move, text="Home", 
            command=self.returnHome)
        move_x.pack(side="left", padx=(3,pad_x_button-aa))
        move_y.pack(side="left", padx=pad_x_button-aa)
        move_z.pack(side="left", padx=pad_x_button-aa)
        home.pack(side='right', padx=20)

        aa = 5
        move_xn = tk.Button(frame_buttons_mid2, text="Move  -X", 
            command= lambda: self.move(sisk.X, False))
        move_yn = tk.Button(frame_buttons_mid2, text="Move  -Y", 
            command= lambda: self.move(sisk.Y, False))
        move_zn = tk.Button(frame_buttons_mid2, text="Move  -Z", 
            command= lambda: self.move(sisk.Z, False))
        move_xn.pack(side="left", padx=(3,pad_x_button-aa))
        move_yn.pack(side="left", padx=pad_x_button-aa)
        move_zn.pack(side="left", padx=pad_x_button-aa)

        stop_x = tk.Button(frame_buttons_bot, text="Stop X", 
            command= lambda: self.stopMove(sisk.X))
        stop_y = tk.Button(frame_buttons_bot, text="Stop Y", 
            command= lambda: self.stopMove(sisk.Y))
        stop_z = tk.Button(frame_buttons_bot, text="Stop Z", 
            command= lambda: self.stopMove(sisk.Z))
        stop_all = tk.Button(frame_buttons_bot, text="Stop ALL",
            command= lambda: self.stopAll())
        stop_x.pack(side="left", padx=(10,pad_x_button))
        stop_y.pack(side="left", padx=pad_x_button)
        stop_z.pack(side="left", padx=pad_x_button)
        stop_all.pack(side="left", padx=pad_x_button)

        default = tk.Button(container_adv, text="Default", command=self.default)
        flush = tk.Button(container_adv, text="Flush", command=self.flush)
        reset = tk.Button(container_adv, text="RESET", command=self.reset)
        default.pack(side="left")
        flush.pack(side="left", padx=30)
        reset.pack(side="left")

        img = Image.fromarray(np.ones([500,800]))
        imgTk = ImageTk.PhotoImage(img)
        self.image = imgTk
        image_label = tk.Label(image_frame, image=self.image)
        image_label.pack(anchor="e")

        close_button = tk.Button(root_bot, text="Close", command=self.stop)
        close_button.pack()

        self.root = root
        self.ser = ser
        self.image_label = image_label
        self.stop_flag = False
        self.reset_flag1 = False
        self.reset_flag2 = False

    def update(self):
        self.pos_var.set(str(self.pos))
        self.move_var.set(str(self.moving))
        self.lims_var.set(str(self.lims))
        self.stat_var.set(str(self.raw_status))
        self.vel_var.set(str(self.vel))
        self.image_label.configure(image=self.image)
        self.root.update()
        if self.stop_flag:
            return True
        else:
            if self.reset_flag1:
                if com.isPathComplete(self.ser):
                    self.reset_p1()
                    self.reset_flag1 = False
                    self.reset_flag2 = True
            elif self.reset_flag2:
                if com.isPathComplete(self.ser):
                    self.reset_p2()
                    self.reset_flag2 = False
            return False

    def zero(self, axis):
        com.setHome(axis, self.ser)

    def zeroAll(self):
        self.zero(sisk.X)
        self.zero(sisk.Y)
        self.zero(sisk.Z)

    def move(self, axis, dir):
        if not self.reset_flag1 or self.reset_flag2:
            if dir:
                com.velocityMode(axis, self.ser, sp, ac)
                self.setVelocity(axis, sp)
            else:
                com.velocityMode(axis, self.ser, -sp, ac)
                self.setVelocity(axis, -sp)
        else:
            print "Currently in reset mode"

    def stopMove(self, axis):
        com.velocityModeDisable(axis, self.ser, self.getVelocity(axis), ac)
        self.setVelocity(axis, 0)
        self.reset_flag1 = False
        self.reset_flag2 = False

    def stopAll(self):
        self.stopMove(sisk.X)
        self.stopMove(sisk.Y)
        self.stopMove(sisk.Z)

    def returnHome(self):
        com.returnHome(sisk.X, self.ser, sp, ac)
        com.returnHome(sisk.Y, self.ser, sp, ac)
        com.returnHome(sisk.Z, self.ser, sp, ac)

    def default(self):
        com.zeroAll(self.ser, sp, ac)
        self.reset_flag1 = True

    def flush(self):
        self.ser.flush()

    def reset(self):
        com.resetAxis(sisk.X, self.ser)
        com.resetAxis(sisk.Y, self.ser)
        com.resetAxis(sisk.Z, self.ser)

    def setPosition(self, pos):
        self.pos = pos

    def setMoving(self, move):
        self.moving = move

    def setLimits(self, lims):
        self.lims = lims

    def setStatus(self, stat):
        self.raw_status = stat

    def start(self):
        self.root.mainloop()

    def stop(self):
        self.stop_flag = True
        self.reset_flag1 = False
        self.reset_flag2 = False

    def reset_p1(self):
        com.moveRelative(sisk.X, self.ser, 2000000, sp, ac)
        com.moveRelative(sisk.Y, self.ser, 2000000, sp, ac)
        com.moveRelative(sisk.Z, self.ser, 2000000, sp, ac)

    def reset_p2(self):
        com.setHome(sisk.X, self.ser)
        com.setHome(sisk.Y, self.ser)
        com.setHome(sisk.Z, self.ser)

    def getVelocity(self, axis):
        if axis is sisk.X:
            return self.vel[0]
        elif axis is sisk.Y:
            return self.vel[1]
        else:
            return self.vel[2]

    def setVelocity(self, axis, amt):
        if axis is sisk.X:
            self.vel = (amt, self.vel[1], self.vel[2])
        elif axis is sisk.Y:
            self.vel = (self.vel[0], amt, self.vel[2])
        else:
            self.vel = (self.vel[0], self.vel[1], amt)

    def setImage(self, img):
        img = Image.fromarray(img)
        img = ImageTk.PhotoImage(img)
        self.image = img
        

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