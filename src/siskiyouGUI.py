#!/usr/bin/env python

import Tkinter as tk
import siskiyouCommands as com
import siskiyouLibrary as sisk
import time

vel = 1000
accel = 100
class Window:
    def __init__(self, ser):
        root = tk.Tk()
        root.title("Manipulator Status")
        root.resizable(width=False, height=False)
        root.geometry("{}x{}".format(1200,700))

        font = ("TkDefaultFont",12)
        pad_y = 5

        container = tk.Frame(root)
        container2 = tk.Frame(root)
        container_adv = tk.Frame(root)
        container.pack(anchor="w")
        container2.pack(anchor="w")
        container_adv.pack(anchor="w",pady=(10,0))

        spacing_y = 5
        frame_text = tk.Frame(container)
        frame_value = tk.Frame(container)
        frame_buttons_top = tk.Frame(container2)
        frame_buttons_mid = tk.Frame(container2)
        frame_buttons_bot = tk.Frame(container2)
        frame_text.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_value.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_buttons_top.pack(anchor="w", pady=spacing_y)
        frame_buttons_mid.pack(anchor="w", pady=spacing_y)
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
        status = tk.Label(frame_value, textvariable=self.stat_var, font=font)
        velocity = tk.Label(frame_value, textvariable=self.vel_var, font=font)
        xyz.pack(anchor="w", pady=pad_y)
        position.pack(anchor="w", pady=pad_y)
        moving.pack(anchor="w", pady=pad_y)
        limits.pack(anchor="w", pady=pad_y)
        status.pack(anchor="w", pady=pad_y)
        velocity.pack(anchor="w", pady=pad_y)

        pad_x_button = 10
        zero_x = tk.Button(frame_buttons_top, text="Zero X", 
            command= lambda: self.zero(sisk.X))      
        zero_y = tk.Button(frame_buttons_top, text="Zero Y", 
            command= lambda: self.zero(sisk.Y))       
        zero_z = tk.Button(frame_buttons_top, text="Zero Z", 
            command= lambda: self.zero(sisk.Z))        
        zero_all = tk.Button(frame_buttons_top, text="Zero ALL",
            command= lambda: self.zeroAll())
        zero_x.pack(side="left", padx=(5,pad_x_button))
        zero_y.pack(side="left", padx=pad_x_button)
        zero_z.pack(side="left", padx=pad_x_button)
        zero_all.pack(side="left", padx=pad_x_button)

        aa = 2
        move_x = tk.Button(frame_buttons_mid, text="Move X", 
            command= lambda: self.move(sisk.X))
        move_y = tk.Button(frame_buttons_mid, text="Move Y", 
            command= lambda: self.move(sisk.Y))
        move_z = tk.Button(frame_buttons_mid, text="Move Z", 
            command= lambda: self.move(sisk.Z))
        home = tk.Button(frame_buttons_mid, text="Home", 
            command=self.returnHome)
        move_x.pack(side="left", padx=(3,pad_x_button-aa))
        move_y.pack(side="left", padx=pad_x_button-aa)
        move_z.pack(side="left", padx=pad_x_button-aa)
        home.pack(side="left", padx=pad_x_button+10)

        stop_x = tk.Button(frame_buttons_bot, text="Stop X", 
            command= lambda: self.stopMove(sisk.X))
        stop_y = tk.Button(frame_buttons_bot, text="Stop Y", 
            command= lambda: self.stopMove(sisk.Y))
        stop_z = tk.Button(frame_buttons_bot, text="Stop Z", 
            command= lambda: self.stopMove(sisk.Z))
        stop_all = tk.Button(frame_buttons_bot, text="Stop ALL",
            command= lambda: self.stopAll())
        stop_x.pack(side="left", padx=(5,pad_x_button))
        stop_y.pack(side="left", padx=pad_x_button)
        stop_z.pack(side="left", padx=pad_x_button)
        stop_all.pack(side="left", padx=pad_x_button)

        reset = tk.Button(container_adv, text="Reset", command=self.reset)
        flush = tk.Button(container_adv, text="Flush", command=self.flush)
        reset.pack(side="left")
        flush.pack(side="left", padx=30)

        close_button = tk.Button(root, text="Close", command=self.stop)
        close_button.pack(side="bottom", pady=(0,5))

        self.root = root
        self.ser = ser
        self.stop_flag = False
        self.reset_flag1 = False
        self.reset_flag2 = False

    def update(self):
        self.pos_var.set(str(self.pos))
        self.move_var.set(str(self.moving))
        self.lims_var.set(str(self.lims))
        self.stat_var.set(str(self.raw_status))
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

    def move(self, axis):
        if not self.reset_flag1 or self.reset_flag2:
            com.velocityMode(axis, self.ser, vel, accel)
        else:
            print "Currently in reset mode"

    def stopMove(self, axis):
        com.velocityModeDisable(axis, self.ser)
        self.reset_flag1 = False
        self.reset_flag2 = False

    def stopAll(self):
        self.stopMove(sisk.X)
        self.stopMove(sisk.Y)
        self.stopMove(sisk.Z)

    def returnHome(self):
        com.returnHome(sisk.X, self.ser, vel, accel)
        com.returnHome(sisk.Y, self.ser, vel, accel)
        com.returnHome(sisk.Z, self.ser, vel, accel)

    def reset(self):
        com.zeroAll(self.ser, vel, accel)
        self.reset_flag1 = True

    def flush(self):
        self.ser.flush()

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
        com.moveRelative(sisk.X, self.ser, 2000000, vel, accel)
        com.moveRelative(sisk.Y, self.ser, 2000000, vel, accel)
        com.moveRelative(sisk.Z, self.ser, 2000000, vel, accel)

    def reset_p2(self):
        com.setHome(sisk.X, self.ser)
        com.setHome(sisk.Y, self.ser)
        com.setHome(sisk.Z, self.ser)

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