#!/usr/bin/env python

import Tkinter as tk
import siskiyouCommands as com
import siskiyouLibrary as sisk
import time

vel = 2000
accel = 100
class Window:
    def __init__(self, ser):
        root = tk.Tk()
        root.title("Manipulator Status")
        root.resizable(width=False, height=False)
        root.geometry("{}x{}".format(700,500))

        font = ("TkDefaultFont",12)
        pad_y = 5

        container = tk.Frame(root)
        container.pack(anchor="w")
        container2 = tk.Frame(root)
        container2.pack()
        container_adv = tk.Frame(root)
        container_adv.pack(pady=(20,0))

        spacing_y = 10
        frame_text = tk.Frame(container)
        frame_text.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_value = tk.Frame(container)
        frame_value.pack(side="left",fill="y",pady=(0,spacing_y))
        frame_buttons_top = tk.Frame(container2)
        frame_buttons_top.pack(pady=spacing_y)
        frame_buttons_mid = tk.Frame(container2)
        frame_buttons_mid.pack(pady=spacing_y)
        frame_buttons_bot = tk.Frame(container2)
        frame_buttons_bot.pack(pady=spacing_y)

        text = tk.Label(frame_text, text= "", font=font)
        text.pack(anchor="w", pady=pad_y)
        text1 = tk.Label(frame_text, text="Position: ", font=font)
        text1.pack(anchor="w", pady=pad_y)
        text2 = tk.Label(frame_text, text="Moving: ", font=font)
        text2.pack(anchor="w", pady=pad_y)
        text3 = tk.Label(frame_text, text="Limits: ", font=font)
        text3.pack(anchor="w", pady=pad_y)
        text4 = tk.Label(frame_text, text="Status: ", font=font)
        text4.pack(anchor="w", pady=pad_y)
        text5 = tk.Label(frame_text, text="Velocity: ", font=font)
        text5.pack(anchor="w", pady=pad_y)

        self.pos = (0, 0, 0)
        self.pos_var = tk.StringVar()
        self.pos_var.set(str(self.pos))
        self.moving = (False, False, False)
        self.move_var = tk.StringVar()
        self.move_var.set(str(self.moving))
        self.status = (False, False, False)
        self.stat_var = tk.StringVar()
        self.stat_var.set(str(self.status))
        self.raw = ('0101000100001010', '0101000100001010', '0101000100001010')
        self.raw_var = tk.StringVar()
        self.raw_var.set(str(self.raw))
        self.vel = (0, 0, 0)
        self.vel_var = tk.StringVar()
        self.vel_var.set(str(self.vel))

        xyz = tk.Label(frame_value, text="(X, Y, Z)", font=font)
        xyz.pack(anchor="w", pady=pad_y)
        position = tk.Label(frame_value, textvariable=self.pos_var, font=font)
        position.pack(anchor="w", pady=pad_y)
        moving = tk.Label(frame_value, textvariable=self.move_var, font=font)
        moving.pack(anchor="w", pady=pad_y)
        limits = tk.Label(frame_value, textvariable=self.stat_var, font=font)
        limits.pack(anchor="w", pady=pad_y)
        status = tk.Label(frame_value, textvariable=self.raw_var, font=font)
        status.pack(anchor="w", pady=pad_y)
        velocity = tk.Label(frame_value, textvariable=self.vel_var, font=font)
        velocity.pack(anchor="w", pady=pad_y)

        pad_x_button = 40
        zero_x = tk.Button(frame_buttons_top, text="Zero X", 
            com= lambda: self.zero(sisk.X))
        zero_x.pack(side="left", padx=pad_x_button)
        zero_y = tk.Button(frame_buttons_top, text="Zero Y", 
            com= lambda: self.zero(sisk.Y))
        zero_y.pack(side="left", padx=pad_x_button)
        zero_z = tk.Button(frame_buttons_top, text="Zero Z", 
            com= lambda: self.zero(sisk.Z))
        zero_z.pack(side="left", padx=pad_x_button)

        move_x = tk.Button(frame_buttons_mid, text="Move X", 
            com= lambda: self.move(sisk.X))
        move_x.pack(side="left", padx=pad_x_button)
        move_y = tk.Button(frame_buttons_mid, text="Move Y", 
            com= lambda: self.move(sisk.Y))
        move_y.pack(side="left", padx=pad_x_button)
        move_z = tk.Button(frame_buttons_mid, text="Move Z", 
            com= lambda: self.move(sisk.Z))
        move_z.pack(side="left", padx=pad_x_button)

        stop_x = tk.Button(frame_buttons_bot, text="Stop X", 
            com= lambda: self.stopMove(sisk.X))
        stop_x.pack(side="left", padx=pad_x_button)
        stop_y = tk.Button(frame_buttons_bot, text="Stop Y", 
            com= lambda: self.stopMove(sisk.Y))
        stop_y.pack(side="left", padx=pad_x_button)
        stop_z = tk.Button(frame_buttons_bot, text="Stop Z", 
            com= lambda: self.stopMove(sisk.Z))
        stop_z.pack(side="left", padx=pad_x_button)

        home = tk.Button(container_adv, text="Return Home", com=self.returnHome)
        home.pack(side="left", padx=20)
        reset = tk.Button(container_adv, text="Reset", com=self.reset)
        reset.pack(side="left", padx=45)

        close_button = tk.Button(root, text="Close", com=self.stop)
        close_button.pack(side="bottom", pady=(0,5))

        frame_value.grid_rowconfigure(1, minsize=50)
        frame_value.grid_columnconfigure(1, minsize=20)

        self.root = root
        self.ser = ser
        self.stop_flag = False
        self.reset_flag1 = False
        self.reset_flag2 = False

    def update(self):
        self.pos_var.set(str(self.pos))
        self.root.update()
        if self.stop_flag:
            return True
        else:
            if reset_flag1:
                if com.isPathComplete(self.ser):
                    self.reset_p1()
                    reset_flag1 = False
                    reset_flag2 = True
            else if reset_flag2:
                if com.isPathComplete(self.ser):
                    self.reset_p2()
                    reset_flag2 = False
            return False

    def zero(self, axis):
        print "Zero", axis

    def move(self, axis):
        if not reset_flag1 or reset_flag2:
            com.velocityMode(axis, self.ser, vel, accel)
        else:
            print "Currently in reset mode"

    def stopMove(self, axis):
        com.velocityModeDisable(axis, self.ser)
        self.reset_flag1 = False
        self.reset_flag2 = False

    def returnHome(self):
        print "Returning Home"

    def reset(self):
        com.zeroAll(ser, vel, accel)
        self.reset_flag1 = True

    def setPosition(self, pos):
        self.pos = pos

    def setMoving(self, move):
        self.moving = move

    def setLimits(self, lims):
        self.limits = lims

    def setStatus(self, stat):
        self.status = stat

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