#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

from Tkinter import Tk, N, W, E, S
import ttk
import rospyoubot
import test_rospyoubot


def key_press(event):
    if event.char == 'q':
        exit()
    if event.char == 'i':
        R1.base.set_velocity(1, 0, 0)
    elif event.char == 'k':
        R1.base.set_velocity(-1, 0, 0)
    elif event.char == 'j':
        R1.base.set_velocity(0, 1, 0)
    elif event.char == 'l':
        R1.base.set_velocity(0, -1, 0)
    elif event.char == 'u':
        R1.base.set_velocity(0, 0, 1)
    elif event.char == 'o':
        R1.base.set_velocity(0, 0, -1)

def key_released(event):
    R1.base.set_velocity()


R1 = rospyoubot.YouBot()
# Main window
root = Tk()
root.title("youBot control")
root.bind("<Key>", key_press)
root.bind("<KeyRelease>", key_released)
# main grid
arm_control = ttk.Frame(root, padding="5 5 5 5", borderwidth=1, relief='solid')
arm_control.grid(column=0, row=0, sticky=(N, W, E, S))
arm_control.columnconfigure(0, weight=1)
arm_control.rowconfigure(0, weight=1)
base_control = ttk.Frame(root, padding="5 5 5 5", borderwidth=1, relief='solid')
base_control.grid(column=0, row=1, sticky=(N, W, E, S))
base_control.columnconfigure(0, weight=1)
base_control.rowconfigure(0, weight=1)
# Joints control
# A1
ttk.Label(arm_control, text='A1:').grid(column=0, row=0, sticky=E)
ttk.Button(arm_control, text='-').grid(column=1, row=0, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=0, sticky=N)
ttk.Button(arm_control, text='+').grid(column=3, row=0, sticky=W)
# A2
ttk.Label(arm_control, text='A2:').grid(column=0, row=1, sticky=E)
ttk.Button(arm_control, text='-').grid(column=1, row=1, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=1, sticky=N)
ttk.Button(arm_control, text='+').grid(column=3, row=1, sticky=W)
# A3
ttk.Label(arm_control, text='A3:').grid(column=0, row=2, sticky=E)
ttk.Button(arm_control, text='-').grid(column=1, row=2, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=2, sticky=N)
ttk.Button(arm_control, text='+').grid(column=3, row=2, sticky=W)
# A4
ttk.Label(arm_control, text='A4:').grid(column=0, row=3, sticky=E)
ttk.Button(arm_control, text='-').grid(column=1, row=3, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=3, sticky=N)
ttk.Button(arm_control, text='+').grid(column=3, row=3, sticky=W)
# A5
ttk.Label(arm_control, text='A5:').grid(column=0, row=4, sticky=E)
ttk.Button(arm_control, text='-').grid(column=1, row=4, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=4, sticky=N)
ttk.Button(arm_control, text='+').grid(column=3, row=4, sticky=W)
#Gripper control
ttk.Label(arm_control, text='Gripper:').grid(column=0, row=5, sticky=E)
ttk.Button(arm_control, text='open').grid(column=1, row=5, sticky=W)
ttk.Label(arm_control, text='state').grid(column=2, row=5, sticky=N)
ttk.Button(arm_control, text='close').grid(column=3, row=5, sticky=W)
# Arm controls pading
for child in arm_control.winfo_children():
    child.grid_configure(padx=2, pady=2)
#Base control
ttk.Button(base_control, text='RL').grid(column=0, row=6, sticky=W)
ttk.Button(base_control, text='F').grid(column=1, row=6, sticky=W)
ttk.Button(base_control, text='RR').grid(column=2, row=6, sticky=W)
ttk.Button(base_control, text='L').grid(column=0, row=7, sticky=W)
ttk.Button(base_control, text='B').grid(column=1, row=7, sticky=W)
ttk.Button(base_control, text='R').grid(column=2, row=7, sticky=W)
# Base controls pading
for child in base_control.winfo_children():
    child.grid_configure(padx=2, pady=2)
# Test button
ttk.Button(root, text="test", command=test_rospyoubot.test).grid(column=0, row=2)

root.mainloop()
