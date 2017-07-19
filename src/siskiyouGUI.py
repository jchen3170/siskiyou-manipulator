#!/usr/bin/env python

import Tkinter as tk

class Window:
    def __init__(self, master):
        self.master = master
        master.title("A simple GUI")

        self.position = (0, 0, 0)

        self.label_pos = tk.StringVar()
        self.label_pos.set("asdf")

        self.text = tk.Label(master, textvariable=self.label_pos)
        self.text.grid(columnspan=4, sticky=tk.W)

        self.greet_button = tk.Button(master, text="Greet", command=self.greet)
        self.greet_button.grid(row=1)

        self.close_button = tk.Button(master, text="Close", command=master.quit)
        self.close_button.grid(row=2,column=5)

    def greet(self):
        print self.label_pos
        self.updateAll()

    def updateAll(self):
        self.label_pos.set("qwer")

root = tk.Tk()
gui = Window(root)
root.mainloop()