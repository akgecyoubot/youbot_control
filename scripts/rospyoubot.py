#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for ROS wrapper for KUKA youBot API."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
from sensor_msgs.msg import JointState
from math import cos, sin, atan, radians, sqrt, pi, acos



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
        rospy.on_shutdown(self.emergencyStop)

    def emergencyStop(self):
        """Stop youBot, and move arm to default position."""
        self.base.set_velocity(0, 0, 0)
        self.arm.set_joints_velocities(0, 0, 0, 0, 0)
        # self.arm.set_joints_angles(0.0100693,
                                   # 0.0100693,
                                   # -0.015708,
                                   # 0.0221239,
                                   # 0.11062)

class Base(object):

    """Control youBot Base."""

    def __init__(self):
        u"""Class constructor.

        Конструктор класса.
        """
        self.velocity = Twist()
        self.odometry = Odometry()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        self.rate = rospy.Rate(10)
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
        mark = -1 if self.odometry.pose.pose.orientation.z < 0 else 1
        position += mark * 2 * acos(self.odometry.pose.pose.orientation.w),
        return position

    def lin(self, Xwp, Ywp, Phip, speed=0.5):  # TODO: Make it work with different speeds
        u"""
        Move youBot base to the point with coordinate (Xwp, Ywp).

        Xwp, Ywp - Coordinates in odometry coordinate system (Meteres)
        Phip - Angle between Odometry X axis and robot X axis (Radians)
        """
        psi = 0.05
        Xwr, Ywr, Phir = self.get_odometry()
        while (abs(Xwp - Xwr) >= psi or abs(Ywp - Ywr) >= psi) and not rospy.is_shutdown():
            Xwr, Ywr, Phir = self.get_odometry()
            print "World coordinates: X={}, Y={}, Phi={}".format(Xwr, Ywr, Phir)
            Xyp, Yyp = _transformCoordinates(Xwp, Ywp, Xwr, Ywr, Phir)
            print "Robroot coordinates: X={}, Y={}".format(Xyp, Yyp)
            Vx, Vy = _calculateVelocity(Xyp, Yyp)
            Vx *= speed
            Vy *= speed
            print "Velocities: Vx={}, Vy={}".format(Vx, Vy)
            print '____________________________________________________________'
            self.set_velocity(Vx, Vy, 0)
            # self.rate.sleep()
        self.set_velocity(0, 0, 0)
        while abs(Phip - Phir) >= psi and not rospy.is_shutdown():
            # Обновляем текущие координаты
            Xwr, Ywr, Phir = self.get_odometry()
            print "Goal orientation: {}".format(Phip)
            # вычислаяем скорость
            print 'Current orientation: ', Phir
            ang_z = _calculateAngularVelocity(Phir, Phip)
            print "Velocity: {}".format(ang_z)
            print '____________________________________________________________'
            # Отправляем сообщение с вычесленной скоростью
            self.set_velocity(0, 0, ang_z)
            # self.rate.sleep()
        self.set_velocity(0, 0, 0)

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

    def set_joints_angles(self, *args):  # TODO: remove *args
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

    def set_joints_velocities(self, *args):  # TODO: remove *args
        u"""Set velocity for each joint.

        Arguments:
            *args -- velocity for each joint (j1, j2, j3, j4, j5)

        Устанавливает скорость каждой степени подвижности в радианах/с.

        Аргументы:
            *args -- скорости соотвествующих степеней (j1, j2, j3, j4, j5)
        """
        assert len(args) == 5
        self.joints_velocities.velocities = []
        for i in range(len(args)):  # TODO: rewrite using enumerate()
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

    def ptp(self, x, y, z, w, ori, elbow):
        u"""Передвигает манипулятор в заданную точку пространства.

        Принимает координаты и ориентацию схвата.
        возвращает углы поворота осей в радианах
        """
        Q = _jointsAnglesForPose(x, y, z, w, ori, elbow)
        if _checkPose(Q, x, y, z):
            self.set_joints_angles(*Q)
        else:
            print 'Woops!'

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

def _jointsAnglesForPose(x, y, z, w, ori, elbow):
    u"""Просчитывает положения степеней подвижности для заданного положения."""
    def a1_calc(x, y, ori):
        u"""Расчет первой степени подвижности."""
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
        u"""Расчет второй степени подвижности."""
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
        u"""Расчет третьей степени подвижности."""
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
        u"""Расчет четвертой степени подвижности."""
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
    a5 = 0
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
    for i in range(5):  # сделать, чтобы проверка осуществлялась до отправки сообщения
        Q3.append(Q0[i] + Q1[i])
    return Q3      # если надо - тупо вызовешь all_ax_calc и он отдаст координаты от свечки в радианах
def _calculateAngularVelocity(current, goal):
    if current > goal:
        return -1
    elif current < goal:
        return 1
    else:
        return 0
def _calculateVelocity(*args):  # TODO: remove *args
    """Return velocity vector."""
    # TODO: Исправить вычисление скорости, чтобы робот не ездил по диагонали
    velocity = ()
    try:
        multiplier = 1 / sqrt(pow(args[0], 2) + pow(args[1],2))
    except ZeroDivisionError:
        multiplier = 0
    velocity += (args[0] * multiplier),
    velocity += (args[1] * multiplier),
    '''
    for v in args:
        if abs(v) > 1:
            velocity.append(v/abs(v))
        elif 0 < abs(v) <= 1:
            velocity.append(round(v, 2))
        else:
            velocity.append(0)
    '''
    return velocity

def _transformCoordinates(Xwp, Ywp, Xwr, Ywr, Phir):
    Xwp -= Xwr
    Ywp -= Ywr
    Xyp = Xwp * cos(Phir) + Ywp * sin(Phir)
    Yyp = -1 * Xwp * sin(Phir) + Ywp * cos(Phir)
    # Xyp = Xwp * cos(Phir) + Ywp * sin(Phir) - Xwr
    # Yyp = -1 * Xwp * sin(Phir) + Ywp * cos(Phir) - Ywr
    return Xyp, Yyp

def _checkPose(Q3, x, y, z):
    u"""Проверяет заданное положение манипулятора."""
    if (0.0100692 <= Q3[0] <= 5.84014 and   # сделать это с промощью перехвата исключений
        0.0100692 <= Q3[1] <= 2.61799 and
        -5.0221239 <= Q3[2] <= -0.015708 and
        0.0221239 <= Q3[3] <= 3.4292 and
        0.1106200 <= Q3[4] <= 5.64159):
        if not(z < 0 and x < 150 and -150 < y < 150):
        # ограничения размера тележки
            return True
        else:
            return False
    else:
        return False
