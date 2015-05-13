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
        self.joints_controls.grid(column=1, row=0, rowspan=2, sticky='nswe')
        #
        self.joints_controls = CurrentPositionFrame(self)
        self.joints_controls.grid(column=1, row=2, sticky='nswe')
        #
        self.odometry = OdometryFrame(self)
        self.odometry.grid(column=0, row=1, rowspan=1, sticky='nswe')
        #
        self.base_control = BaseControl(self)
        self.base_control.grid(column=0, row=0, sticky='nswe')
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
        self.angle = tk.StringVar()     # Эта строчка видимо лишняя
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

class CurrentPositionFrame(ttk.LabelFrame):
    def __init__(self, parent):
        ttk.LabelFrame.__init__(self, parent, text='Координаты схвата в СК базы:')
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.AXIS_POSITION = (tk.StringVar)

        ttk.Label(self, text='X:', width=6, anchor='e').grid(column=0, row=0)
        ttk.Label(self, text='Y:', width=6, anchor='e').grid(column=0, row=1)
        ttk.Label(self, text='Z:', width=6, anchor='e').grid(column=0, row=2)

        self.x_value = ttk.Label(self, textvariable=self.AXIS_POSITION[0], width=10, anchor='center')
        self.x_value.grid(column=1, row=0)
        self.y_value = ttk.Label(self, textvariable=self.AXIS_POSITION[1], width=10, anchor='center')
        self.y_value.grid(column=1, row=1)
        self.z_value = ttk.Label(self, textvariable=self.AXIS_POSITION[2], width=10, anchor='center')
        self.z_value.grid(column=1, row=2)

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
        self.points_list = tk.Listbox(self,
                                      height=26,
                                      selectmode='browse',
                                      listvariable=self.pt_list)
        self.points_list.grid(column=0,
                              row=0,
                              sticky='nswe',
                              rowspan=2,
                              columnspan=2)
        self.buttons_frame = ttk.Frame(self)
        self.buttons_frame.grid(column=2, row=0, sticky='n')
        self.add_button = ttk.Button(self.buttons_frame,
                                     text=u'Добавить',
                                     width=7)
        self.add_button.grid(column=0, row=0)
        self.add_button.bind('<Button-1>', self.add_to_list)
        ttk.Button(self.buttons_frame,
                   text=u'Редактировать',
                   width=7).grid(column=0, row=1)
        self.remove_button = ttk.Button(self.buttons_frame,
                                        text=u'Удалить',
                                        width=7)
        self.remove_button.grid(column=0, row=2)
        self.remove_button.bind('<Button-1>', self.remove_point)
        ttk.Button(self, text=u'Вниз').grid(column=0, row=2)
        ttk.Button(self, text=u'Вверх').grid(column=1, row=2)
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)
        for child in self.buttons_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

    def add_to_list(self, *args):
        u"""Добавляет движение в список движений."""
        window = MotionAdditionWindow(self)
        points = self.pt_list.get()
        points = points[1:-1]
        points = points.split()
        for i in range(len(points)):
            points[i] = points[i].strip(",'")
        points.append('New_item')
        points = ' '.join(points)
        self.pt_list.set(points)

    def remove_point(self, *args):
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

class MotionAdditionWindow(tk.Toplevel):

    u"""Окно добавления движения."""

    def __init__(self, parent):
        u"""Инициализация класса."""
        tk.Toplevel.__init__(self, parent)
        self.title(u'Выбор движения')
        self.resizable(0, 0)
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(column=0, row=0, sticky='nswe')
        self.base_motion = BaseMotionAddition(self)
        self.notebook.add(self.base_motion, text=u'Платформа')

class BaseMotionAddition(ttk.Frame):
    def __init__(self, parent):
        ttk.Frame.__init__(self, parent)
        self.grid(column=0, row=0)
        ttk.Label(self,
                  text=u'Имя:').grid(column=0, row=0, sticky='e')
        ttk.Entry(self).grid(column=1, row=0, sticky='w')
        ttk.Label(self,
                  text=u'Тип движения:').grid(column=2, row=0, sticky='e')
        ttk.Combobox(self).grid(column=3, row=0)
        ttk.Label(self,
                  text=u'P1:',
                  width=3).grid(column=4, row=0, sticky='e')
        ttk.Entry(self).grid(column=5, row=0, sticky='w')
        ttk.Button(self, text='Touch Up').grid(column=5, row=1)
        ttk.Label(self,
                  text=u'P2:',
                  width=3).grid(column=6, row=0, sticky='e')
        ttk.Entry(self).grid(column=7, row=0, sticky='w')
        ttk.Button(self, text='Touch Up').grid(column=7, row=1)
        ttk.Button(self, text=u'Сохранить').grid(row=2, column=0)
        cancel_button = ttk.Button(self,
                                   text=u'Отмена',
                                   command=self.cancel)
        cancel_button.grid(row=2, column=1)
        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5)

    def cancel(self, *args):
        u"""Закрывает окно, не сохраняя результат."""
        self.destroy()

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
    for i in range(3):
        AXIS_POSITION[i].set(round(current_joints_positions[i], 3))
    ROOT.after(100, update_joints_labels)


if __name__ == '__main__':
    ROOT = tk.Tk()
    ROOT.title("youBot control")
    ROOT.resizable(1, 0)
    ROOT.columnconfigure(0, weight=1)
    BASE_VELOCITY = 1
    ARM_VELOCITY = 1
    R1 = rospyoubot.YouBot()
    ARM_JOINTS_ANGLES = [tk.StringVar() for i in range(5)]  # с чем связана цифра
    ODOMETRY = [tk.StringVar() for i in range(3)]
    AXIS_POSITION = [tk.StringVar() for i in range(3)]
    MAINFRAME = MainApplication(ROOT)
    ROOT.update()
    ROOT.minsize(ROOT.winfo_width(), ROOT.winfo_height())
    ROOT.bind('<Key>', key_pressed)
    ROOT.bind('<KeyRelease>', key_released)
    ROOT.after(100, update_joints_labels)
    ROOT.mainloop()
