#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

import Tkinter as tk
import ttk
import rospyoubot


BASE_VELOCITY = 1
ARM_VELOCITY = 1
R1 = rospyoubot.YouBot()
class MainApplication(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        for i in range(5):
            arm_joints_angles.append(tk.StringVar())
        for i in range(3):
            odometry.append(tk.StringVar())
        self.bind('<Key>', key_pressed)
        self.bind('<KeyRelease>', key_released)
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.title("youBot control")
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(column=0, row=0, sticky=(tk.N, tk.S, tk.W, tk.E))
        self.f1 = ControlsPage(self.notebook)
        self.notebook.add(self.f1, text='Controls')

class ControlsPage(ttk.Frame):
    def __init__(self, parent):
        ttk.Frame.__init__(self, parent)
        self.odometry = OdometryFrame(self)
        self.odometry.grid(column=0, row=0, rowspan=2, sticky=(tk.N, tk.S))
        self.joints_controls = JointsControlsFrame(self)
        self.joints_controls.grid(column=1, row=0, sticky=(tk.E, tk.W))
        self.base_control = BaseControl(self)
        self.base_control.grid(column=1, row=1, sticky=(tk.W, tk.E))
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)

class OdometryFrame(ttk.LabelFrame):
    def __init__(self, parent):
        self.odom_x = tk.StringVar()
        self.odom_x.set('x')
        self.odom_y = tk.StringVar()
        self.odom_y.set('y')
        self.odom_z = tk.StringVar()
        self.odom_z.set('z')
        ttk.LabelFrame.__init__(self, parent, text='Odometry:')
        ttk.Label(self, text='X:').grid(column=0, row=0)
        ttk.Label(self, textvariable=odometry[0]).grid(column=1, row=0)
        ttk.Label(self, text='Y:').grid(column=0, row=1)
        ttk.Label(self, textvariable=odometry[1]).grid(column=1, row=1)
        ttk.Label(self, text='ori Z:').grid(column=0, row=2)
        ttk.Label(self, textvariable=odometry[2]).grid(column=1, row=2)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

class JointsControlsFrame(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Joints control:')
        a1 = JointControl(self, 1)
        a1.grid(row=0, column=0)
        a2 = JointControl(self, 2)
        a2.grid(row=1, column=0)
        a3 = JointControl(self, 3)
        a3.grid(row=2, column=0)
        a4 = JointControl(self, 4)
        a4.grid(row=3, column=0)
        a5 = JointControl(self, 5)
        a5.grid(row=4, column=0)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

class JointControl(ttk.Frame):
    def __init__(self, parent, joint):
        ttk.Frame.__init__(self, parent)
        self.joint = joint
        self.label = 'A{}:'.format(joint)
        self.angle = tk.StringVar()
        ttk.Label(self, text=self.label).grid(column=0, row=0)
        self.minus_button = ttk.Button(self, text='-', width=1)
        self.minus_button.grid(column=1, row=0)
        self.minus_button.bind('<Button-1>', self.minus_button_press)
        self.minus_button.bind('<ButtonRelease-1>', key_released)
        ttk.Label(self, textvariable=arm_joints_angles[joint-1]).grid(column=2, row=0)
        self.plus_button = ttk.Button(self, text='+', width=1)
        self.plus_button.grid(column=3, row=0)
        self.plus_button.bind('<Button-1>', self.plus_button_press)
        self.plus_button.bind('<ButtonRelease-1>', key_released)
        self.angle.set('state')

    def plus_button_press(self, *args):
        arm_velocities = [ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

    def minus_button_press(self, *args):
        arm_velocities = [-ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

class BaseControl(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Base control:')
        # Rotate left
        self.rl_button = ttk.Button(self, text='RL', width=2)
        self.rl_button.grid(column=0, row=0, sticky=tk.SE)
        self.rl_button.bind('<Button-1>', self.rl_button_press)
        self.rl_button.bind('<ButtonRelease-1>', key_released)
        # Forward
        self.f_button = ttk.Button(self, text='F', width=2)
        self.f_button.grid(column=1, row=0, sticky=tk.S)
        self.f_button.bind('<Button-1>', self.f_button_press)
        self.f_button.bind('<ButtonRelease-1>', key_released)
        # Rotate right
        self.rr_button = ttk.Button(self, text='RR', width=2)
        self.rr_button.grid(column=2, row=0, sticky=tk.SW)
        self.rr_button.bind('<Button-1>', self.rr_button_press)
        self.rr_button.bind('<ButtonRelease-1>', key_released)
        # Left
        self.l_button = ttk.Button(self, text='L', width=2)
        self.l_button.grid(column=0, row=1, sticky=tk.NE)
        self.l_button.bind('<Button-1>', self.l_button_press)
        self.l_button.bind('<ButtonRelease-1>', key_released)
        # Backwards
        self.b_button = ttk.Button(self, text='B', width=2)
        self.b_button.grid(column=1, row=1, sticky=tk.N)
        self.b_button.bind('<Button-1>', self.b_button_press)
        self.b_button.bind('<ButtonRelease-1>', key_released)
        # Right
        self.r_button = ttk.Button(self, text='R', width=2)
        self.r_button.grid(column=2, row=1, sticky=tk.NW)
        self.r_button.bind('<Button-1>', self.r_button_press)
        self.r_button.bind('<ButtonRelease-1>', key_released)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

    def rl_button_press(self, *args):
        R1.base.set_velocity(ang_z=BASE_VELOCITY)

    def f_button_press(self, *args):
        R1.base.set_velocity(lin_x=BASE_VELOCITY)

    def l_button_press(self, *args):
        R1.base.set_velocity(lin_y=BASE_VELOCITY)

    def r_button_press(self, *args):
        R1.base.set_velocity(lin_y=-BASE_VELOCITY)

    def b_button_press(self, *args):
        R1.base.set_velocity(lin_x=-BASE_VELOCITY)

    def rr_button_press(self, *args):
        R1.base.set_velocity(ang_z=-BASE_VELOCITY)

def key_pressed(event):
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

def key_released(event):
    R1.base.set_velocity()
    R1.arm.set_joints_velocities(0, 0, 0, 0, 0)

def update_joints_labels():
    current_joints_positions = list(R1.arm.get_current_joints_positions())
    odom = R1.base.get_odometry()
    for i in range(3):
        odometry[i].set(round(odom[i], 3))
    for i in range(5):
        current_joints_positions[i] = round(current_joints_positions[i], 3)
        arm_joints_angles[i].set(current_joints_positions[i])
    APP.after(100, update_joints_labels)


if __name__ == '__main__':
    arm_joints_angles = []
    odometry = []
    APP = MainApplication()
    APP.after(100, update_joints_labels)
    APP.mainloop()
