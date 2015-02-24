#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

import Tkinter as tk
import ttk


class MainApplication(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
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
        ttk.Label(self, textvariable=self.odom_x).grid(column=1, row=0)
        ttk.Label(self, text='Y:').grid(column=0, row=1)
        ttk.Label(self, textvariable=self.odom_y).grid(column=1, row=1)
        ttk.Label(self, text='ori Z:').grid(column=0, row=2)
        ttk.Label(self, textvariable=self.odom_z).grid(column=1, row=2)
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
        self.label = 'A{}:'.format(joint)
        self.angle = tk.StringVar()
        ttk.Label(self, text=self.label).grid(column=0, row=0)
        self.minus_button = ttk.Button(self, text='-', width=1)
        self.minus_button.grid(column=1, row=0)
        ttk.Label(self, textvariable=self.angle).grid(column=2, row=0)
        self.plus_button = ttk.Button(self, text='+', width=1)
        self.plus_button.grid(column=3, row=0)
        self.angle.set('state')

class BaseControl(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Base control:')
        ttk.Button(self, text='RL', width=2).grid(column=0, row=0, sticky=tk.SE)
        ttk.Button(self, text='F', width=2).grid(column=1, row=0, sticky=tk.S)
        ttk.Button(self, text='RR', width=2).grid(column=2, row=0, sticky=tk.SW)
        ttk.Button(self, text='L', width=2).grid(column=0, row=1, sticky=tk.NE)
        ttk.Button(self, text='B', width=2).grid(column=1, row=1, sticky=tk.N)
        ttk.Button(self, text='R', width=2).grid(column=2, row=1, sticky=tk.NW)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)


if __name__ == '__main__':
    app = MainApplication()
    app.mainloop()
