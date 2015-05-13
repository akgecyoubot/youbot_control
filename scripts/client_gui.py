#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""Client GUI to control youBot robot."""

import Tkinter as tk
import ttk
import rospyoubot


class MainApplication(ttk.Frame):

    u"""Основное окно приложения."""

    def __init__(self, parent, *args, **kwargs):
        u"""Конструктор основного окна."""
        ttk.Frame.__init__(self, parent, *args, **kwargs)
        self.grid(sticky='nswe')
        self.columnconfigure(0, weight=1)
        # self.columnconfigure(1, weight=1)
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(column=0, row=0, sticky='nswe')
        self.manual_controls = ControlsPage(self.notebook)
        self.notebook.add(self.manual_controls,
                          text='Ручное управление',
                          sticky='nswe')
        self.automatic_controls = AutomaticControls(self.notebook)
        self.notebook.add(self.automatic_controls,
                          text='Автоматическое управление',
                          sticky='nswe')

class ControlsPage(ttk.Frame):

    u"""Вкладка управления."""

    def __init__(self, parent):
        u"""Конструктор класса."""
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        #
        self.joints_controls = JointsControlsFrame(self)
        self.joints_controls.grid(column=1, row=0, sticky='nswe')
        #
        self.odometry = OdometryFrame(self)
        self.odometry.grid(column=1, row=1, sticky='nswe')
        #
        self.base_control = BaseControl(self)
        self.base_control.grid(column=1, row=2, sticky='nswe')
        #
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)

class OdometryFrame(ttk.LabelFrame):

    u"""Фрейм одометрии."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        ttk.LabelFrame.__init__(self, parent, text='Одометрия:')
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
        ttk.Label(self, text=u'\u03c6:', width=5, anchor=tk.E).grid(column=0,
                                                                    row=2)
        ttk.Label(self,
                  textvariable=ODOMETRY[2],
                  width=6,
                  anchor=tk.W).grid(column=1, row=2)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

class JointsControlsFrame(ttk.LabelFrame):

    u"""Фрейм управления степенями подвижности."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        ttk.LabelFrame.__init__(self, parent, text='Управление манипулятором:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.a1_joint = JointControl(self, 1)
        self.a1_joint.grid(row=0, columnspan=2, sticky='nswe')
        self.a2_joint = JointControl(self, 2)
        self.a2_joint.grid(row=1, columnspan=2, sticky='nswe')
        self.a3_joint = JointControl(self, 3)
        self.a3_joint.grid(row=2, columnspan=2, sticky='nswe')
        self.a4_joint = JointControl(self, 4)
        self.a4_joint.grid(row=3, columnspan=2, sticky='nswe')
        self.a5_joint = JointControl(self, 5)
        self.a5_joint.grid(row=4, columnspan=2, sticky='nswe')
        self.gripper = GripperControl(self)
        self.gripper.grid(row=5, columnspan=2, sticky='nswe')
        self.home_button = ttk.Button(self, text='Домой', width=6)
        self.home_button.grid(row=6, column=0, sticky='nswe')
        self.home_button.bind('<Button-1>', self.go_home)
        self.home_button = ttk.Button(self, text='Свеча', width=6)
        self.home_button.grid(row=6, column=1, sticky='nswe')
        self.home_button.bind('<Button-1>', self.go_candle)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

    def go_home(self, *args):
        u"""Отправляет манипулятор в домашнюю позицию."""
        R1.arm.set_joints_angles(0.0100693,
                                 0.0100693,
                                 -0.015708,
                                 0.0221239,
                                 0.11062)

    def go_candle(self, *args):
        u"""Приводит манипулятор в положение свечки."""
        R1.arm.set_joints_angles(2.9400474018133402,
                                 1.1251030074812907,
                                 -2.5235000069592695,
                                 1.769468876296561,
                                 2.838871440356912)

class JointControl(ttk.Frame):

    u"""Фрейм управления отдельной степенью."""

    def __init__(self, parent, joint):
        u"""Инициализация класса."""
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)
        self.joint = joint
        self.label = 'A{}:'.format(joint)
        self.angle = tk.StringVar()
        ttk.Label(self, text=self.label, width=6, anchor='e').grid(column=0,
                                                                   row=0,
                                                                   sticky=tk.E)
        self.minus_button = ttk.Button(self, text='-', width=7)
        self.minus_button.grid(column=1, row=0)
        self.minus_button.bind('<Button-1>', self.minus_button_press)
        self.minus_button.bind('<ButtonRelease-1>', key_released)
        self.state_label = ttk.Label(self,
                                     textvariable=ARM_JOINTS_ANGLES[joint-1],
                                     width=5,
                                     anchor=tk.CENTER)
        self.state_label.grid(column=2, row=0, sticky='nswe')
        self.plus_button = ttk.Button(self, text='+', width=7)
        self.plus_button.grid(column=3, row=0)
        self.plus_button.bind('<Button-1>', self.plus_button_press)
        self.plus_button.bind('<ButtonRelease-1>', key_released)

    def plus_button_press(self, *args):
        u"""Задаёт скорость оси, при нажатии на кнопку '+'."""
        arm_velocities = [ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

    def minus_button_press(self, *args):
        u"""Задаёт скорость оси, при нажатии на кнопку '-'."""
        arm_velocities = [-ARM_VELOCITY if x == self.joint - 1 else 0 for x in range(5)]
        R1.arm.set_joints_velocities(*arm_velocities)

class BaseControl(ttk.LabelFrame):

    u"""Фрейм управления движением базы."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        ttk.LabelFrame.__init__(self, parent, text='Управление платформой:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        controls_style = ttk.Style()
        controls_style.configure('base.TButton', font=('TkDefaultFont', 20))
        # Rotate left
        self.rl_button = ttk.Button(self, text=u'\u21b6', width=2, style='base.TButton')
        self.rl_button.grid(column=0, row=0, sticky=tk.SE)
        self.rl_button.bind('<Button-1>', self.rl_button_press)
        self.rl_button.bind('<ButtonRelease-1>', key_released)
        # Forward
        self.f_button = ttk.Button(self, text=u'\u2191', width=2, style='base.TButton')
        self.f_button.grid(column=1, row=0, sticky=tk.S)
        self.f_button.bind('<Button-1>', self.f_button_press)
        self.f_button.bind('<ButtonRelease-1>', key_released)
        # Rotate right
        self.rr_button = ttk.Button(self, text=u'\u21b7', width=2, style='base.TButton')
        self.rr_button.grid(column=2, row=0, sticky=tk.SW)
        self.rr_button.bind('<Button-1>', self.rr_button_press)
        self.rr_button.bind('<ButtonRelease-1>', key_released)
        # Left
        self.l_button = ttk.Button(self, text=u'\u2190', width=2, style='base.TButton')
        self.l_button.grid(column=0, row=1, sticky=tk.NE)
        self.l_button.bind('<Button-1>', self.l_button_press)
        self.l_button.bind('<ButtonRelease-1>', key_released)
        # Backwards
        self.b_button = ttk.Button(self, text=u'\u2193', width=2, style='base.TButton')
        self.b_button.grid(column=1, row=1, sticky=tk.N)
        self.b_button.bind('<Button-1>', self.b_button_press)
        self.b_button.bind('<ButtonRelease-1>', key_released)
        # Right
        self.r_button = ttk.Button(self, text=u'\u2192', width=2, style='base.TButton')
        self.r_button.grid(column=2, row=1, sticky=tk.NW)
        self.r_button.bind('<Button-1>', self.r_button_press)
        self.r_button.bind('<ButtonRelease-1>', key_released)
        for child in self.winfo_children():
            child.grid_configure(padx=2, pady=2)

    def rl_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку RL."""
        R1.base.set_velocity(ang_z=BASE_VELOCITY)

    def f_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку F."""
        R1.base.set_velocity(lin_x=BASE_VELOCITY)

    def l_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку L."""
        R1.base.set_velocity(lin_y=BASE_VELOCITY)

    def r_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку R."""
        R1.base.set_velocity(lin_y=-BASE_VELOCITY)

    def b_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку B."""
        R1.base.set_velocity(lin_x=-BASE_VELOCITY)

    def rr_button_press(self, *args):
        u"""Обрабатыевает нажатие на кнопку RR."""
        R1.base.set_velocity(ang_z=-BASE_VELOCITY)

class GripperControl(ttk.Frame):

    u"""Фрейм управления гриппером."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)
        self.gripper_state = tk.StringVar()
        ttk.Label(self, text='Схват:', width=6, anchor='e').grid(column=0,
                                                                 row=0,
                                                                 sticky='e')
        self.close_button = ttk.Button(self, text='Закрыть', width=7)
        self.close_button.grid(column=1, row=0)
        self.close_button.bind('<Button-1>', self.close_gripper)
        ttk.Label(self,
                  textvariable=self.gripper_state,
                  anchor=tk.CENTER,
                  width=5).grid(column=2, row=0, sticky=(tk.W, tk.E))
        self.open_button = ttk.Button(self, text='Открыть', width=7)
        self.open_button.grid(column=3, row=0)
        self.open_button.bind('<Button-1>', self.open_gripper)

    def close_gripper(self, *args):
        u"""Закрывает гриппер и записывает 'Closed' в его статус."""
        self.gripper_state.set('Закрыт')
        R1.arm.gripper.set_gripper_state(False)

    def open_gripper(self, *args):
        u"""Открывает гриппер и записывает 'Opened' в его статус."""
        self.gripper_state.set('Открыт')
        R1.arm.gripper.set_gripper_state(True)

class AutomaticControls(ttk.Frame):

    u"""Фрейм автоматического управления."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        ttk.Frame.__init__(self, parent)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.pt_list = tk.StringVar()
        # Points Listbox
        self.points_list = tk.Listbox(self,
                                      height=26,
                                      selectmode='browse',
                                      listvariable=self.pt_list)
        self.points_list.grid(column=0,
                              row=0,
                              sticky='nswe',
                              rowspan=2,
                              columnspan=2)
        # Buttons frame
        self.buttons_frame = ttk.Frame(self)
        self.buttons_frame.grid(column=2, row=0, sticky='n')
        # Add button
        self.add_button = ttk.Button(self.buttons_frame,
                                     text=u'Платформа',
                                     width=9)
        self.add_button.grid(column=0, row=0)
        self.add_button.bind('<Button-1>', self.add_to_list)
        # Edit button
        ttk.Button(self.buttons_frame,
                   text=u'Редактировать',
                   width=9).grid(column=0, row=1)
        # Remove button
        self.remove_button = ttk.Button(self.buttons_frame,
                                        text=u'Удалить',
                                        width=9)
        self.remove_button.grid(column=0, row=2)
        self.remove_button.bind('<Button-1>', self.remove_point)
        # Start button
        ttk.Button(self.buttons_frame,
                   text=u'Старт',
                   width=9,
                   command=self.start).grid(column=0, row=3)
        # Stop button
        ttk.Button(self.buttons_frame, text=u'Стоп', width=9).grid(column=0, row=4)
        ttk.Button(self, text=u'Вниз').grid(column=0, row=2)
        ttk.Button(self, text=u'Вверх').grid(column=1, row=2)
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)
        for child in self.buttons_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

    def add_to_list(self, event):
        u"""Добавляет движение в список движений."""
        window = BaseMotionAddition(self)

    def remove_point(self, event):
        u"""Удаляет выбранное движение из списка."""
        if len(self.points_list.curselection()) > 0:
            index = int(self.points_list.curselection()[0])
            points = self.pt_list.get()
            points = points[1:-1]
            points = points.split()
            for i in range(len(points)):
                points[i] = points[i].strip(",'")
            points.pop(index)
            points = ' '.join(points)
            self.pt_list.set(points)

    def start(self):
        for key in sorted(POINTS_DICT.keys()):
            print "Goint to point {}, with coordinates {}".format(key, POINTS_DICT[key])
            R1.base.lin(*POINTS_DICT[key])
            R1.base.set_velocity(0, 0, 0)

class BaseMotionAddition(tk.Toplevel):

    u"""Окно добавления движения."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        tk.Toplevel.__init__(self, parent)
        self.parent = parent
        self.title(u'Движение платформы')
        self.resizable(0, 0)
        self.frm = ttk.Frame(self)
        self.frm.grid(column=0, row=0, sticky='nswe')
        ttk.Label(self.frm,
                  text=u'Имя точки:').grid(column=0, row=0, sticky='e')
        # Point's name
        self.point_name = tk.StringVar()
        ttk.Entry(self.frm,
                  textvariable=self.point_name).grid(column=1,
                                                     row=0,
                                                     sticky='w')
        # X coordinate
        ttk.Label(self.frm,
                  text=u'X:',
                  width=3).grid(column=3, row=0, sticky='e')
        self.X = ttk.Entry(self.frm)
        self.X.grid(column=4, row=0, sticky='w')
        # Y coordinate
        ttk.Label(self.frm,
                  text=u'Y:',
                  width=3).grid(column=3, row=1, sticky='e')
        self.Y = ttk.Entry(self.frm)
        self.Y.grid(column=4, row=1, sticky='w')
        # Orientation
        ttk.Label(self.frm,
                  text=u'\u03c6:',
                  width=3).grid(column=3, row=2, sticky='e')
        self.Phi = ttk.Entry(self.frm)
        self.Phi.grid(column=4, row=2, sticky='w')
        # Touch Up! button
        ttk.Button(self.frm,
                   text='Touch Up',
                   command=self.touch_up).grid(column=4, row=3)
        # Save button
        save_button = ttk.Button(self.frm, text=u'Сохранить', command=self.save)
        save_button.grid(row=3, column=0)
        # Cancel button
        cancel_button = ttk.Button(self.frm,
                                   text=u'Отмена',
                                   command=self.cancel)
        cancel_button.grid(row=3, column=1)
        for child in self.frm.winfo_children():
            child.grid_configure(padx=5, pady=5)

    def cancel(self):
        u"""Закрывает окно, не сохраняя результат."""
        self.destroy()

    def save(self):
        u"""Сохраняет точку в список точек."""
        points = self.parent.pt_list.get()[1:-1]
        points_list = points.split()
        for i in range(len(points_list)):
            points_list[i] = points_list[i].strip(",'")
        name = 'Base:{}'.format(self.point_name.get())
        x = self.X.get()
        y = self.Y.get()
        phi = self.Phi.get()
        POINTS_DICT[name] = (float(x), float(y), float(phi))
        points_list.append(name)
        points = ' '.join(points_list)
        self.parent.pt_list.set(points)
        self.destroy()

    def touch_up(self):
        odometry = R1.base.get_odometry()
        self.X.insert(0, odometry[0])
        self.Y.insert(0, odometry[1])
        self.Phi.insert(0, odometry[2])
        pass
def key_pressed(event):
    u"""Обрабатывает нажатие на кнопку клавиатуры."""
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
    if event.char == 'q':
        R1.arm.set_joints_velocities(1, 0, 0, 0, 0)
    elif event.char == 'Q':
        R1.arm.set_joints_velocities(-1, 0, 0, 0, 0)
    if event.char == 'w':
        R1.arm.set_joints_velocities(0, 1, 0, 0, 0)
    elif event.char == 'W':
        R1.arm.set_joints_velocities(0, -1, 0, 0, 0)
    if event.char == 'e':
        R1.arm.set_joints_velocities(0, 0, 1, 0, 0)
    elif event.char == 'E':
        R1.arm.set_joints_velocities(0, 0, -1, 0, 0)
    if event.char == 'r':
        R1.arm.set_joints_velocities(0, 0, 0, 1, 0)
    elif event.char == 'R':
        R1.arm.set_joints_velocities(0, 0, 0, -1, 0)
    if event.char == 't':
        R1.arm.set_joints_velocities(0, 0, 0, 0, 1)
    elif event.char == 'T':
        R1.arm.set_joints_velocities(0, 0, 0, 0, -1)
    if event.char == 'G':
        R1.arm.gripper.set_gripper_state(True)
    if event.char == 'g':
        R1.arm.gripper.set_gripper_state(False)

def key_released(event):
    u"""Обрабатывает отпускание кнопки клавиатуры."""
    R1.base.set_velocity()
    R1.arm.set_joints_velocities(0, 0, 0, 0, 0)

def update_joints_labels():
    u"""бновляет данные о текущем угле поворота осей и одометрии базы."""
    current_joints_positions = list(R1.arm.get_current_joints_positions())
    odom = R1.base.get_odometry()
    for i in range(3):
        ODOMETRY[i].set(round(odom[i], 3))
    for i in range(5):
        ARM_JOINTS_ANGLES[i].set(round(current_joints_positions[i], 3))
    ROOT.after(100, update_joints_labels)


if __name__ == '__main__':
    ROOT = tk.Tk()
    ROOT.title("youBot control")
    ROOT.resizable(1, 0)
    ROOT.columnconfigure(0, weight=1)
    BASE_VELOCITY = 1
    ARM_VELOCITY = 1
    R1 = rospyoubot.YouBot()
    ARM_JOINTS_ANGLES = [tk.StringVar() for i in range(5)]
    ODOMETRY = [tk.StringVar() for i in range(3)]
    POINTS_DICT = {}
    MAINFRAME = MainApplication(ROOT)
    ROOT.update()
    ROOT.minsize(ROOT.winfo_width(), ROOT.winfo_height())
    ROOT.bind('<Key>', key_pressed)
    ROOT.bind('<KeyRelease>', key_released)
    ROOT.after(100, update_joints_labels)
    ROOT.mainloop()
