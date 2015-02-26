#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

import Tkinter as tk
import ttk
import rospyoubot


class MainApplication(ttk.Frame):
    def __init__(self, parent, *args, **kwargs):
        ttk.Frame.__init__(self, parent, *args, **kwargs)
        self.grid(sticky='nswe')
        self.columnconfigure(0, weight=1)
        # self.columnconfigure(1, weight=1)
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(column=0, row=0, sticky='nswe')
        self.f1 = ControlsPage(self.notebook)
        self.notebook.add(self.f1, text='Controls', sticky='nswe')

class ControlsPage(ttk.Frame):
    def __init__(self, parent):
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.joints_controls = JointsControlsFrame(self)
        self.joints_controls.grid(column=1, row=0, sticky='nswe')
        self.odometry = OdometryFrame(self)
        self.odometry.grid(column=1, row=1, rowspan=1, sticky='nswe')
        self.base_control = BaseControl(self)
        self.base_control.grid(column=1, row=2, sticky='nswe')
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)

class OdometryFrame(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Odometry:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.odom_x = tk.StringVar()
        self.odom_x.set('x')
        self.odom_y = tk.StringVar()
        self.odom_y.set('y')
        self.odom_z = tk.StringVar()
        self.odom_z.set('z')
        ttk.Label(self, text='X:', width=5, anchor=tk.E).grid(column=0, row=0)
        ttk.Label(self,
                  textvariable=ODOMETRY[0],
                  width=6,
                  anchor=tk.W).grid(column=1, row=0)
        ttk.Label(self, text='Y:', width=5, anchor=tk.E).grid(column=0, row=1)
        ttk.Label(self,
                  textvariable=ODOMETRY[1],
                  width=6,
                  anchor=tk.W).grid(column=1, row=1)
        ttk.Label(self, text='ori Z:', width=5, anchor=tk.E).grid(column=0,
                                                                  row=2)
        ttk.Label(self,
                  textvariable=ODOMETRY[2],
                  width=6,
                  anchor=tk.W).grid(column=1, row=2)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

class JointsControlsFrame(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Joints control:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.a1 = JointControl(self, 1)
        self.a1.grid(row=0, columnspan=2, sticky='nswe')
        self.a2 = JointControl(self, 2)
        self.a2.grid(row=1, columnspan=2, sticky='nswe')
        self.a3 = JointControl(self, 3)
        self.a3.grid(row=2, columnspan=2, sticky='nswe')
        self.a4 = JointControl(self, 4)
        self.a4.grid(row=3, columnspan=2, sticky='nswe')
        self.a5 = JointControl(self, 5)
        self.a5.grid(row=4, columnspan=2, sticky='nswe')
        self.gripper = GripperControl(self)
        self.gripper.grid(row=5, columnspan=2, sticky='nswe')
        self.home_button = ttk.Button(self, text='HOME', width=6)
        self.home_button.grid(row=6, column=0, sticky='nswe')
        self.home_button.bind('<Button-1>', go_home)
        self.home_button = ttk.Button(self, text='CANDLE', width=6)
        self.home_button.grid(row=6, column=1, sticky='nswe')
        self.home_button.bind('<Button-1>', go_candle)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

class JointControl(ttk.Frame):
    def __init__(self, parent, joint):
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)
        self.joint = joint
        self.label = 'A{}:'.format(joint)
        self.angle = tk.StringVar()
        ttk.Label(self, text=self.label, width=3).grid(column=0, row=0, sticky=tk.W)
        self.minus_button = ttk.Button(self, text='-', width=1)
        self.minus_button.grid(column=1, row=0, sticky=tk.W)
        self.minus_button.bind('<Button-1>', self.minus_button_press)
        self.minus_button.bind('<ButtonRelease-1>', key_released)
        self.state_label = ttk.Label(self,
                                     textvariable=ARM_JOINTS_ANGLES[joint-1],
                                     width=5,
                                     anchor=tk.CENTER)
        self.state_label.grid(column=2, row=0, sticky=(tk.W, tk.E))
        self.plus_button = ttk.Button(self, text='+', width=1)
        self.plus_button.grid(column=3, row=0, sticky=tk.E)
        self.plus_button.bind('<Button-1>', self.plus_button_press)
        self.plus_button.bind('<ButtonRelease-1>', key_released)

    def plus_button_press(self, *args):
        arm_velocities = [ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

    def minus_button_press(self, *args):
        arm_velocities = [-ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

class BaseControl(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Base control:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
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

class GripperControl(ttk.Frame):
    def __init__(self, parent):
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)
        ttk.Label(self, text='G:', width=3).grid(column=0, row=0, sticky='w')
        self.close_button = ttk.Button(self, text='C', width=1)
        self.close_button.grid(column=1, row=0, sticky='w')
        self.close_button.bind('<Button-1>', close_gripper)
        ttk.Label(self,
                  textvariable=GRIPPER_STATE,
                  anchor=tk.CENTER,
                  width=5).grid(column=2, row=0, sticky=(tk.W, tk.E))
        self.open_button = ttk.Button(self, text='O', width=1)
        self.open_button.grid(column=3, row=0, sticky='e')
        self.open_button.bind('<Button-1>', open_gripper)

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
        ODOMETRY[i].set(round(odom[i], 3))
    for i in range(5):
        ARM_JOINTS_ANGLES[i].set(round(current_joints_positions[i], 3))
    ROOT.after(100, update_joints_labels)

def go_home(*args):
    R1.arm.set_joints_angles(0.0100693, 0.0100693, -0.015708, 0.0221239, 0.11062)

def go_candle(*args):
    R1.arm.set_joints_angles(2.95, 1.1, -2.6, 1.8, 2.95)

def close_gripper(*args):
    GRIPPER_STATE.set('Close')
    R1.arm.gripper.set_gripper_state(False)

def open_gripper(*args):
    GRIPPER_STATE.set('Open')
    R1.arm.gripper.set_gripper_state(True)


if __name__ == '__main__':
    BASE_VELOCITY = 1
    ARM_VELOCITY = 1
    R1 = rospyoubot.YouBot()
    ROOT = tk.Tk()
    ROOT.title("youBot control")
    ROOT.resizable(1, 0)
    ROOT.columnconfigure(0, weight=1)
    ARM_JOINTS_ANGLES = [tk.StringVar() for i in range(5)]
    ODOMETRY = [tk.StringVar() for i in range(3)]
    GRIPPER_STATE = tk.StringVar()
    MAINFRAME = MainApplication(ROOT)
    ROOT.update()
    ROOT.minsize(ROOT.winfo_width(), ROOT.winfo_height())
    ROOT.bind('<Key>', key_pressed)
    ROOT.bind('<KeyRelease>', key_released)
    ROOT.after(100, update_joints_labels)
    ROOT.mainloop()
