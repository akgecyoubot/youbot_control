#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

from Tkinter import Tk, N, W, E, S, NW, NE, SE, SW, StringVar
import ttk
import rospyoubot


def key_press(event):
    # update_joints_labels()
    # Base movement
    if event.char == 'i':
        R1.base.set_velocity(lin_x=1)
    elif event.char == 'k':
        R1.base.set_velocity(lin_x=-1)
    elif event.char == 'j':
        R1.base.set_velocity(lin_y=1)
    elif event.char == 'l':
        R1.base.set_velocity(lin_y=-1)
    elif event.char == 'u':
        R1.base.set_velocity(ang_z=1)
    elif event.char == 'o':
        R1.base.set_velocity(ang_z=-1)
    # Arm movement
    if event.char == '1':
        R1.arm.set_joints_velocities(1)
    elif event.char == 'q':
        R1.arm.set_joints_velocities(-1)

def key_released(event):
    R1.base.set_velocity()
    R1.arm.set_joints_velocities(0, 0, 0, 0, 0)

def drive_forward(event):
    R1.base.set_velocity(lin_x=0.5)

def update_joints_labels():
    current_joints_positions = list(R1.arm.get_current_joints_positions())
    odom = R1.base.get_odometry()
    for i in range(3):
        odometry[i].set(round(odom[i], 3))
    for i in range(5):
        current_joints_positions[i] = round(current_joints_positions[i], 3)
        arm_joints_angles[i].set(current_joints_positions[i])
    root.after(100, update_joints_labels)

def close_gripper():
    R1.arm.gripper.set_gripper_state(open_gripper=False)

def open_gripper():
    R1.arm.gripper.set_gripper_state(open_gripper=True)


if __name__ == '__main__':
    R1 = rospyoubot.YouBot()
    # Main window
    root = Tk()
    root.columnconfigure(0, weight=2)
    root.title("youBot control")
    root.bind("<Key>", key_press)
    root.bind("<KeyRelease>", key_released)
    style = ttk.Style()
    style.theme_use('clam')  # aqua, step, clam, alt, default, classic
    arm_joints_angles = []
    for i in range(5):
        arm_joints_angles.append(StringVar())
    # Status display
    odometry = []
    for i in range(3):
        odometry.append(StringVar())
    # Info grid
    status = ttk.LabelFrame(root, text='Info:', padding="5 5 5 5", borderwidth=0, relief='solid')
    status.grid(column=0, row=0, sticky=(N, W, E, S))
    ttk.Label(status, text='Odometry').grid(row=0)
    ttk.Label(status, text='X:').grid(column=0, row=1)
    ttk.Label(status, textvariable=odometry[0]).grid(column=1, row=1)
    ttk.Label(status, text='Y:').grid(column=0, row=2)
    ttk.Label(status, textvariable=odometry[1]).grid(column=1, row=2)
    ttk.Label(status, text='ori Z:').grid(column=0, row=3)
    ttk.Label(status, textvariable=odometry[2]).grid(column=1, row=3)
    # Arm control grid
    arm_control = ttk.LabelFrame(root, text='Arm controls:', padding="5 5 5 5", borderwidth=0, relief='solid')
    arm_control.grid(column=1, row=0, sticky=(N, W, E, S))
    arm_control.columnconfigure(0, weight=1)
    arm_control.rowconfigure(0, weight=1)
    # Base control grid
    base_control = ttk.LabelFrame(root, text='Base controls:', padding="5 5 5 5", borderwidth=0, relief='solid')
    base_control.grid(column=1, row=1, sticky=(N, W, E, S))
    base_control.columnconfigure(0, weight=1)
    base_control.rowconfigure(0, weight=1)
    # Test button
    # ttk.Button(root, text="test", command=test_rospyoubot.test).grid(column=1, row=2)
    # Joints control
    # A1
    ttk.Label(arm_control, text='A1:').grid(column=0, row=0, sticky=E)
    ttk.Button(arm_control, text='-').grid(column=1, row=0, sticky=W)
    ttk.Label(arm_control, textvariable=arm_joints_angles[0]).grid(column=2, row=0, sticky=N)
    ttk.Button(arm_control, text='+').grid(column=3, row=0, sticky=W)
    # A2
    ttk.Label(arm_control, text='A2:').grid(column=0, row=1, sticky=E)
    ttk.Button(arm_control, text='-').grid(column=1, row=1, sticky=W)
    ttk.Label(arm_control, textvariable=arm_joints_angles[1]).grid(column=2, row=1, sticky=N)
    ttk.Button(arm_control, text='+').grid(column=3, row=1, sticky=W)
    # A3
    ttk.Label(arm_control, text='A3:').grid(column=0, row=2, sticky=E)
    ttk.Button(arm_control, text='-').grid(column=1, row=2, sticky=W)
    ttk.Label(arm_control, textvariable=arm_joints_angles[2]).grid(column=2, row=2, sticky=N)
    ttk.Button(arm_control, text='+').grid(column=3, row=2, sticky=W)
    # A4
    ttk.Label(arm_control, text='A4:').grid(column=0, row=3, sticky=E)
    ttk.Button(arm_control, text='-').grid(column=1, row=3, sticky=W)
    ttk.Label(arm_control, textvariable=arm_joints_angles[3]).grid(column=2, row=3, sticky=N)
    ttk.Button(arm_control, text='+').grid(column=3, row=3, sticky=W)
    # A5
    ttk.Label(arm_control, text='A5:').grid(column=0, row=4, sticky=E)
    ttk.Button(arm_control, text='-').grid(column=1, row=4, sticky=W)
    ttk.Label(arm_control, textvariable=arm_joints_angles[4]).grid(column=2, row=4, sticky=N)
    ttk.Button(arm_control, text='+').grid(column=3, row=4, sticky=W)
    #Gripper control
    ttk.Label(arm_control, text='Gripper:').grid(column=0, row=5, sticky=E)
    ttk.Button(arm_control, text='O', command=open_gripper).grid(column=1, row=5, sticky=W)
    ttk.Label(arm_control, text='closed').grid(column=2, row=5, sticky=N)
    ttk.Button(arm_control, text='C', command=close_gripper).grid(column=3, row=5, sticky=W)
    # Arm controls pading
    for child in arm_control.winfo_children():
        child.grid_configure(padx=2, pady=2)
    #Base control
    ttk.Button(base_control, text='RL').grid(column=0, row=6, sticky=SE)
    F = ttk.Button(base_control, text='F')
    F.grid(column=1, row=6, sticky=S)
    F.bind("<Button-1>", drive_forward)
    F.bind("<ButtonRelease-1>", key_released)
    ttk.Button(base_control, text='RR').grid(column=2, row=6, sticky=SW)
    ttk.Button(base_control, text='L').grid(column=0, row=7, sticky=NE)
    ttk.Button(base_control, text='B').grid(column=1, row=7, sticky=N)
    ttk.Button(base_control, text='R').grid(column=2, row=7, sticky=NW)
    # Base controls pading
    for child in base_control.winfo_children():
        child.grid_configure(padx=2, pady=2)
    # Main loop
    root.after(100, update_joints_labels())
    root.mainloop()
