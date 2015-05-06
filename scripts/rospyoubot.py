#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for ROS wrapper for KUKA youBot API."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
from sensor_msgs.msg import JointState
from math import cos, sin, atan, radians, degrees, sqrt, pi, acos



class YouBot(object):

    u"""youBot class.

    Attributes:
        arm (YouBot.Arm): controls youBot arm.
        base (YouBot.Base): controls youBot base.

    Класс youBot'a

    Атрибуты:
        arm (YouBot.Arm): контролирует манипулятор
        base (YouBot.Base): контролирует базу

    """

    def __init__(self):
        u"""Class constructor.

        Конструктор класса.
        """
        rospy.init_node('rospyoubot')
        self.arm = Arm()
        self.base = Base()

class Base(object):

    """Control youBot Base."""

    def __init__(self):
        u"""Class constructor.

        Конструктор класса.
        """
        self.velocity = Twist()
        self.odometry = Odometry()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber('/odom', Odometry, self._update_odometry)

    def set_velocity(self, lin_x=0, lin_y=0, ang_z=0):
        u"""Set base velocity vector.

        Arguments:
            lin_x -- linear velocity along X axis (default 0)
            lin_y -- linear velocity along Y axis (default 0)
            ang_z -- angular velocity around Z axis (default 0)

        Задаёт вектор скорости базы.

        Аргументы:
            lin_x -- линейная скорость вдоль оси Х (default 0)
            lin_y -- линейная скорость вдоль оси Y (default 0)
            ang_z -- угловая скорость вдоль оси Z (default 0)
        """
        self.velocity.linear.x = lin_x
        self.velocity.linear.y = lin_y
        self.velocity.angular.z = ang_z
        self.velocity_publisher.publish(self.velocity)

    def _update_odometry(self, data):
        u"""Update odometry every time message is recieved.

        Обновляет одометрию при поступлении сообщения.
        """
        self.odometry = data

    def get_odometry(self):
        u"""Return list with odometry.

        Returns:
            tuple: (X, Y, rotation Z)

        Возвращает одометрию.

        Возвращает:
            кортеж (X, Y, поворот по Z)
        """
        position = ()
        position += self.odometry.pose.pose.position.x,
        position += self.odometry.pose.pose.position.y,
        position += self.odometry.pose.pose.orientation.z,
        return position

    def lin_goto(self, *args):
        u"""Передвигает базу youBot'а в точку с координатами (X,Y,Phi)."""
        speed = 1
        # Задаём погрешность
        psi = 0.1
        # Получаем текущие координаты
        current_position = self.get_odometry()
        # Вычисляем дельту
        delta = [args[i] - current_position[i] for i in range(3)]
        # Пока дельта > psi:
        while abs(delta[0]) >= psi or abs(delta[1]) >= psi:
            # Обновляем текущие координаты
            current_position = self.get_odometry()
            # Обновляем дельту
            delta = [args[i] - current_position[i] for i in range(3)]
            print 'd: ', delta
            # вычислаяем скорость
            velocity = []
            for d in delta:
                if abs(d) > 1:
                    velocity.append(d / abs(d) * speed)
                elif 0 < abs(d) <= 1:
                    velocity.append(d)
                else:
                    velocity.append(0)
            print 'v: ', velocity
            # Отправляем сообщение с вычесленной скоростью
            self.set_velocity(*velocity)
        # Обнуляем скорость
        self.set_velocity()

class Arm(object):

    """Control youBot Arm."""

    def __init__(self):
        u"""Class constructor.

        Конструктор класса.
        """
        self.gripper = Gripper()
        self.joints_positions = JointPositions()
        self.current_joints_states = JointState()
        self.current_joints_states.position = [0.0 for i in range(5)]
        self.joints_velocities = JointVelocities()
        self.joints_positions_publisher = rospy.Publisher('/arm_1/arm_controller/position_command',
                                                          JointPositions)
        self.joints_velocities_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command',
                                                           JointVelocities)
        rospy.Subscriber('/joint_states',
                         JointState,
                         self._update_joints_states)

    def set_joints_angles(self, *args):
        u"""Set arm joints to defined angles in radians.

        Arguments:
            *args -- joints angles (j1, j2, j3, j4, j5)

        Устанавливает углы поворота степеней подвижности в радианах.

        Аргументы:
            *args -- уголы соотвествующих степеней (j1, j2, j3, j4, j5)
        """
        assert len(args) <= 5
        self.joints_positions.positions = []
        for i in range(5):
            tmp = JointValue()
            tmp.timeStamp = rospy.Time.now()
            tmp.joint_uri = 'arm_joint_{}'.format(i + 1)
            tmp.unit = 'rad'
            tmp.value = args[i]
            self.joints_positions.positions.append(tmp)
        self.joints_positions_publisher.publish(self.joints_positions)

    def set_joints_velocities(self, *args):
        u"""Set velocity for each joint.

        Arguments:
            *args -- velocity for each joint (j1, j2, j3, j4, j5)

        Устанавливает скорость каждой степени подвижности в радианах/с.

        Аргументы:
            *args -- скорости соотвествующих степеней (j1, j2, j3, j4, j5)
        """
        assert len(args) == 5
        self.joints_velocities.velocities = []
        for i in range(len(args)):  # I know it's unpythonic, sorry
            tmp = JointValue()
            tmp.timeStamp = rospy.Time.now()
            tmp.joint_uri = 'arm_joint_{}'.format(i + 1)
            tmp.unit = 's^-1 rad'
            tmp.value = args[i]
            self.joints_velocities.velocities.append(tmp)
        self.joints_velocities_publisher.publish(self.joints_velocities)

    def _update_joints_states(self, joints_data):
        """Update joints states when info is received."""
        self.current_joints_states = joints_data

    def get_current_joints_positions(self):
        """Return list of current joints angles."""
        return self.current_joints_states.position

    def ptp(self, x, y, z, w, ori, elbow, a5=0):
        u"""Формирует сообщение установки углов
        для достижения указанной конфигурации

        принимает координаты и ориентацию схвата.
        возвращает углы поворота осей в радианах
        """

        Q = all_axis_calc(x, y, z, w, ori, elbow)
        if self.check(Q, x, y, z):
            self.set_joints_angles(*Q)
        else:
            print 'Woops!'

    def check(self, Q3, x, y, z):
        if (0.0100692 <= Q3[0] <= 5.84014 and   # сделать это с промощью перехвата исключений
            0.0100692 <= Q3[1] <= 2.61799 and
            -0.0221239 <= Q3[2] <= -0.015708 and
            0.0221239 <= Q3[3] <= 3.4292 and
            0.1106200 <= Q3[4] <= 5.64159):
            if z < 0 and x < 150 and -150 < y < 150:
            # ограничения размера тележки
                return True
            else:
                return False
        else:
            return False

class Gripper(object):

    """Gripper class."""

    def __init__(self):
        """Gripper constructor."""
        self.gripper_position = JointPositions()
        self.gripper_position_publisher = rospy.Publisher('arm_1/gripper_controller/position_command',
                                                          JointPositions)

    def set_gripper_state(self, open_gripper=True):
        """Open/close gripper.

        Arguments:
            open_gripper (bool): if True - opens gripper, if False - otherwise
        """
        self.gripper_position.positions = []
        if open_gripper:
            # Open gripper
            tmp_gripper_position_r = JointValue()
            tmp_gripper_position_r.joint_uri = 'gripper_finger_joint_r'
            tmp_gripper_position_r.value = 0.011499
            tmp_gripper_position_r.unit = 'm'
            tmp_gripper_position_r.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_r)
            tmp_gripper_position_l = JointValue()
            tmp_gripper_position_l.joint_uri = 'gripper_finger_joint_l'
            tmp_gripper_position_l.value = 0.011499
            tmp_gripper_position_l.unit = 'm'
            tmp_gripper_position_l.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_l)
        else:
            # Close gripper
            tmp_gripper_position_r = JointValue()
            tmp_gripper_position_r.joint_uri = 'gripper_finger_joint_r'
            tmp_gripper_position_r.value = 0
            tmp_gripper_position_r.unit = 'm'
            tmp_gripper_position_r.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_r)
            tmp_gripper_position_l = JointValue()
            tmp_gripper_position_l.joint_uri = 'gripper_finger_joint_l'
            tmp_gripper_position_l.value = 0
            tmp_gripper_position_l.unit = 'm'
            tmp_gripper_position_l.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_l)
        self.gripper_position_publisher.publish(self.gripper_position)

def all_axis_calc(x, y, z, w, ori, elbow, a5=0):
    def a1_calc(x, y, ori):
        """ расчет первой степени подвижности """
        if ori == 0:
        # 1. ориентация плечо вперед
            if y == 0:
                A1 = 0
            elif x == 0 and y > 0:
                A1 = -pi / 2
            elif x == 0 and y < 0:
                A1 = pi / 2
            elif x > 0:
                A1 = -atan(y / x)
            elif x < 0 and y > 0:
                A1 = -atan(y / x) - pi
            elif x < 0 and y < 0:
                A1 = -atan(y / x) + pi
        else:
        # 2. ориентация плечо назад
            if y == 0:
                A1 = pi
            elif x == 0 and y > 0:
                A1 = pi / 2
            elif x == 0 and y < 0:
                A1 = -pi / 2
            elif x < 0:
                A1 = -atan(y / x)
            elif x > 0 and y >= 0:
                A1 = -atan(y / x) + pi
            elif x > 0 and y < 0:
                A1 = -atan(y / x) - pi
        return A1

    def a2_calc(x, y, z, w, ori, A1, A3):
        """ расчет второй степени подвижности """
        X = x - a * cos(A1)
        Y = y + a * sin(A1)
        beta = atan((L3 * sin(A3)) / (L2 + L3 * cos(A3)))
        alpha_v = ((z - (L4 + L5) * cos(radians(w))) - L1)
        alpha_hp = (sqrt(X ** 2 + Y ** 2) - (L4 + L5) * sin(radians(w)))
        alpha_hm = (-sqrt(X ** 2 + Y ** 2) - (L4 + L5) * sin(radians(w)))

        if ori == 0:
        # ориентация плечо вперед
            if  alpha_v >= 0:
            # если 4-ая степень выше 2-ой
                if sqrt(x ** 2 + y ** 2) >= 33:
                    alpha = atan(alpha_hp / alpha_v)
                else:
                # если точка между осью А1 и осью А2
                    alpha = atan(alpha_hm / alpha_v)
                A2 = alpha - beta
            else:
            # если 4-ая степень опускается ниже 2-ой
                alpha = atan(alpha_v / alpha_hp)
                A2 = pi / 2 - alpha - beta
        else:
        # ориентация плечо назад
            if alpha_v >= 0:
                alpha = atan(alpha_hp / alpha_v)
                A2 = -alpha - beta
            else:
                alpha = atan(alpha_v / alpha_hp)
                A2 = -pi / 2 + alpha - beta
        return A2

    def a3_calc(x, y, z, w, ori, elbow, A1):
        """ расчет третьей степени подвижности """
        X = x - a * cos(A1)
        Y = y + a * sin(A1)
        if ori == 0:                                            # сделать тут try
            if sqrt(x ** 2 + y ** 2) >= 33:
                Cos_A3 = ((z - (L4 + L5) * cos(radians(w)) - L1) ** 2 + (sqrt(X ** 2 + Y ** 2) - (L4 + L5) * sin(radians(w))) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
            else:
                Cos_A3 = ((z - (L4 + L5) * cos(radians(w)) - L1) ** 2 + (-sqrt(X ** 2 + Y ** 2) - (L4 + L5) * sin(radians(w))) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
            if elbow == 0:
            # локоть вверх
                A3 = acos(Cos_A3)
            else:
            # локоть вниз
                A3 = -acos(Cos_A3)
        else:
            Cos_A3 = ((z - (L4 + L5) * cos(radians(w)) - L1) ** 2 + (sqrt(X ** 2 + Y ** 2) - (L4 + L5) * sin(radians(w))) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
            if elbow == 0:
                A3 = -acos(Cos_A3)
            else:
                A3 = acos(Cos_A3)
        return A3

    def a4_calc(w, ori, A2, A3):
        """ расчет четвертой степени подвижности """
        if ori == 0:
            A4l = radians(w) - A2 - A3
        else:
            A4l = -radians(w)- A2 - A3

        if A4l > pi:
        # перевод угла в формат от -pi до pi
            A4 = A4l - 2 * pi
        elif A4l < - pi:
            A4 = A4l + 2 * pi
        else:
            A4 = A4l
        return A4

    L1 = 147.0
    L2 = 155.0
    L3 = 135.0
    L4 = 217.0
    L5 = 0.5
    a = 33.0
    A1 = a1_calc(x, y, ori)
    A3 = a3_calc(x, y, z, w, ori, elbow, A1)
    A2 = a2_calc(x, y, z, w, ori, A1, A3)
    A4 = a4_calc(w, ori, A2, A3)
    A5 = radians(a5)
    a01 = radians(169) - 0.0100693
    a02 = radians(65) - 0.0100693
    a03 = radians(-146) + 0.015708
    a04 = radians(102.5) - 0.0221239
    a05 = radians(169) - 0.11062
    Q0 = [a01, a02, a03, a04, a05]
    Q1 = [A1, A2, A3, A4, A5]
    Q3 = []
    for i in range(5):                          # сделать, чтобы проверка осуществлялась до отправки сообщения
        Q3.append(Q0[i] + Q1[i])
    return Q3      # если надо - тупо вызовешь all_ax_calc и он отдаст координаты от свечки в радианах
