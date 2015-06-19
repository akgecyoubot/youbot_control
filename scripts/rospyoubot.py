#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for ROS wrapper for KUKA youBot API."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
from sensor_msgs.msg import JointState
from math import cos, sin, atan, radians, sqrt, pi, acos, degrees


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
        rospy.on_shutdown(self.go_home)

    def go_home(self):
        """Stop youBot, and move arm to default position."""
        self.base.set_velocity(0, 0, 0)
        self.arm.set_joints_velocities(0, 0, 0, 0, 0)
        self.arm.set_joints_angles(0.0100693,
                                   0.0100693,
                                   -0.015708,
                                   0.0221239,
                                   0.11062)


class Base(object):

    """Control youBot Base."""

    def __init__(self):
        u"""Class constructor.

        Конструктор класса.
        """
        self.velocity = Twist()
        self.odometry = Odometry()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        # self.rate = rospy.Rate(10)
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

    # TODO: Make it work with different speeds
    def lin(self, world_xp, world_yp, goal_ori, speed=0.5):
        u"""Move youBot base to the point with coordinate (world_xp, world_yp).

        world_xp, world_yp - Coordinates in odometry coordinate system (Meteres)
        goal_ori - Angle between Odometry X axis and robot X axis (Radians)
        """
        psi = 0.05
        world_xr, world_yr, current_ori = self.get_odometry()
        # Движение по прямой
        while (abs(world_xp - world_xr) >= psi or abs(world_yp - world_yr) >= psi) and not rospy.is_shutdown():
            world_xr, world_yr, current_ori = self.get_odometry()
            print "World coordinates: X={}, Y={}, Phi={}".format(world_xr,
                                                                 world_yr,
                                                                 current_ori)
            robot_xp, robot_yp = _transform_coordinates(world_xp,
                                                        world_yp,
                                                        world_xr,
                                                        world_yr,
                                                        current_ori)
            print "Robroot coordinates: X={}, Y={}".format(robot_xp, robot_yp)
            velocity_x, velocity_y = _calculate_velocity(robot_xp, robot_yp)
            velocity_x *= speed
            velocity_y *= speed
            print "Velocities: velocity_x={}, velocity_y={}".format(velocity_x,
                                                                    velocity_y)
            print '____________________________________________________________'
            self.set_velocity(velocity_x, velocity_y, 0)
            # self.rate.sleep()
        self.set_velocity(0, 0, 0)
        # Изменение ориентации
        while abs(goal_ori - current_ori) >= psi and not rospy.is_shutdown():
            # Обновляем текущие координаты
            world_xr, world_yr, current_ori = self.get_odometry()
            print "Goal orientation: {}".format(goal_ori)
            # вычислаяем скорость
            print 'Current orientation: ', current_ori
            ang_z = _calculate_angular_velocity(current_ori,
                                                goal_ori)
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
        self.current_joints_states.position = [0.0 for _ in range(5)]
        self.joints_velocities = JointVelocities()
        position_topic = '/arm_1/arm_controller/position_command'
        self.joints_positions_publisher = rospy.Publisher(position_topic,
                                                          JointPositions)
        velocity_topic = '/arm_1/arm_controller/velocity_command'
        self.joints_velocities_publisher = rospy.Publisher(velocity_topic,
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
        for index, value in enumerate(args):
            tmp = JointValue()
            tmp.timeStamp = rospy.Time.now()
            tmp.joint_uri = 'arm_joint_{}'.format(index + 1)
            tmp.unit = 's^-1 rad'
            tmp.value = value
            self.joints_velocities.velocities.append(tmp)
        self.joints_velocities_publisher.publish(self.joints_velocities)

    def _update_joints_states(self, joints_data):
        """Update joints states when info is received."""
        self.current_joints_states = joints_data

    def get_current_joints_positions(self):
        """Return list of current joints angles."""
        return self.current_joints_states.position[:5]

    def ptp(self, joints):
        u"""Передвигает манипулятор в заданную точку пространства.

        Принимает координаты и ориентацию схвата.
        возвращает углы поворота осей в радианах
        """
        goal = [round(x,3) for x in joints]
        current = [round(x,3) for x in self.get_current_joints_positions()]
        condition = goal != current
        while condition and not rospy.is_shutdown():
            print goal, ' | ', current
            self.set_joints_angles(*goal)
            current = [round(x,3) for x in self.get_current_joints_positions()]
            condition = goal != current


class Gripper(object):

    """Gripper class."""

    def __init__(self):
        """Gripper constructor."""
        self.gripper_position = JointPositions()
        gripper_topic = 'arm_1/gripper_controller/position_command'
        self.gripper_position_publisher = rospy.Publisher(gripper_topic,
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


def _joints_angles_for_pose(x, y, z, w, ori, elbow):
    u"""Просчитывает положения степеней подвижности для заданного положения."""
    x = float(x)
    y = float(y)
    z = float(z)
    w = degrees(float(w))

    def calculate_joint1_value(x, y, ori):
        u"""Расчет первой степени подвижности."""
        if ori == 0:
            # 1. ориентация плечо вперед
            if y == 0 and x == 0:
                joint1_value = 0
            elif x == 0 and y > 0:
                joint1_value = -pi / 2
            elif x == 0 and y < 0:
                joint1_value = pi / 2
            elif x > 0:
                joint1_value = -atan(y / x)
            elif x < 0 and y > 0:
                joint1_value = -atan(y / x) - pi
            elif x < 0 and y <= 0:
                joint1_value = -atan(y / x) + pi
        else:
            # 2. ориентация плечо назад
            if y == 0 and x == 0:
                joint1_value = 0
            elif x == 0 and y > 0:
                joint1_value = pi / 2
            elif x == 0 and y < 0:
                joint1_value = -pi / 2
            elif x < 0:
                joint1_value = -atan(y / x)
            elif x > 0 and y >= 0:
                joint1_value = -atan(y / x) + pi
            elif x > 0 and y < 0:
                joint1_value = -atan(y / x) - pi
        return joint1_value

    def calculate_joint2_value(x, y, z, w, ori, joint1_value, joint3_value):
        u"""Расчет второй степени подвижности."""
        X = x - shoulder * cos(joint1_value)
        Y = y + shoulder * sin(joint1_value)
        beta = atan((link3_length * sin(joint3_value)) / (link2_length + link3_length * cos(joint3_value)))
        alpha_v = ((z - (link4_length + link5_length) * cos(radians(w))) - link1_length)
        alpha_hp = (sqrt(X ** 2 + Y ** 2) - (link4_length + link5_length) * sin(radians(w)))
        alpha_hm = (-sqrt(X ** 2 + Y ** 2) - (link4_length + link5_length) * sin(radians(w)))

        if ori == 0:
            if sqrt(x ** 2 + y ** 2) >= 33 and alpha_v <= 0 and alpha_hp >= 33:
                alpha = - atan(alpha_v / alpha_hp) + pi / 2    # наверняка проблема при alpha_hp == 0 - надо проверить как-то

            elif sqrt(x ** 2 + y ** 2) >= 33 and alpha_v <= 0 and alpha_hp < 33:   # это недопустимо роботом, но оставлю пока для проверки
                alpha = - atan(alpha_v / alpha_hp) - pi / 2

            elif sqrt(x ** 2 + y ** 2) < 33 and alpha_v >= 0:
                alpha = atan(alpha_hm / alpha_v)

            elif sqrt(x ** 2 + y ** 2) < 33 and alpha_v < 0:            # это тоже вряд ли возможно
                alpha = - atan(alpha_v / alpha_hm) - pi / 2
            else:
                alpha = atan(alpha_hp / alpha_v)
            joint2_value = alpha - beta
        else:
            if alpha_v <= 0 and alpha_hp >= 0:      # перепроверить при alpha_v == 0, но скорее всего так
                alpha = -atan(alpha_v / alpha_hp) + pi / 2
            elif alpha_v <= 0 and alpha_hp < 0:
                alpha = -atan(alpha_v / alpha_hp) - pi / 2
            else:
                alpha = atan(alpha_hp / alpha_v)
            joint2_value = -alpha - beta
        return joint2_value

    def calculate_joint3_value(x, y, z, w, ori, elbow, joint1_value):
        u"""Расчет третьей степени подвижности."""
        X = x - shoulder * cos(joint1_value)
        Y = y + shoulder * sin(joint1_value)
        if ori == 0:
            if sqrt(x ** 2 + y ** 2) >= 33:
                Cos_joint3_value = (((z - (link4_length + link5_length) * cos(radians(w)) - link1_length) ** 2 +
                                     (sqrt(X ** 2 + Y ** 2) - (link4_length + link5_length) * sin(radians(w))) ** 2 -
                                     link2_length ** 2 - link3_length ** 2) / (2 * link2_length * link3_length))
            else:
                Cos_joint3_value = (((z - (link4_length + link5_length) * cos(radians(w)) - link1_length) ** 2 +
                                     (-sqrt(X ** 2 + Y ** 2) - (link4_length + link5_length) * sin(radians(w))) ** 2 -
                                     link2_length ** 2 - link3_length ** 2) / (2 * link2_length * link3_length))
            if elbow == 0:
                joint3_value = acos(Cos_joint3_value)
            else:
                joint3_value = -acos(Cos_joint3_value)
        else:
            Cos_joint3_value = (((z - (link4_length + link5_length) * cos(radians(w)) - link1_length) ** 2 +
                                 (sqrt(X ** 2 + Y ** 2) - (link4_length + link5_length) * sin(radians(w))) ** 2 -
                                 link2_length ** 2 - link3_length ** 2) / (2 * link2_length * link3_length))
            if elbow == 0:
                joint3_value = -acos(Cos_joint3_value)
            else:
                joint3_value = acos(Cos_joint3_value)
        return joint3_value

    def calculate_joint4_value(w, ori, joint2_value, joint3_value):
        u"""Расчет четвертой степени подвижности."""
        if ori == 0:
            joint4_valuel = radians(w) - joint2_value - joint3_value
        else:
            joint4_valuel = -1 * radians(w) - joint2_value - joint3_value

        if joint4_valuel > pi:
            joint4_value = joint4_valuel - 2 * pi
        elif joint4_valuel < - pi:
            joint4_value = joint4_valuel + 2 * pi
        else:
            joint4_value = joint4_valuel
        return joint4_value

    link1_length = 147.0
    link2_length = 155.0
    link3_length = 135.0
    link4_length = 217.0
    link5_length = 0.5
    shoulder = 33.0
    joint5_rotation = 0  # Degrees
    # Calculating joints values
    joint1_value = calculate_joint1_value(x, y, ori)
    #TODO: всунуть joint3_value в инструкцию try
    joint3_value = calculate_joint3_value(x, y, z, w, ori, elbow, joint1_value)
    joint2_value = calculate_joint2_value(x, y, z, w, ori, joint1_value,
                                          joint3_value)
    joint4_value = calculate_joint4_value(w, ori, joint2_value, joint3_value)
    joint5_value = radians(joint5_rotation)
    joints_values = [joint1_value,
                     joint2_value,
                     joint3_value,
                     joint4_value,
                     joint5_value]
    # Joints corrections
    joint1_correction = radians(169) - 0.0100693
    joint2_correction = radians(65) - 0.0100693
    joint3_correction = radians(-146) + 0.015708
    joint4_correction = radians(102.5) - 0.0221239
    joint5_correction = radians(169) - 0.11062
    joints_correction = [joint1_correction,
                         joint2_correction,
                         joint3_correction,
                         joint4_correction,
                         joint5_correction]
    # Final result
    result = []
    for index, value in enumerate(joints_values):
        correction = joints_correction[index]
        result.append(value + correction)
    if not (0.0100692 <= result[0] <= 5.84014 and
            0.0100692 <= result[1] <= 2.61799 and
            -5.0221239 <= result[2] <= -0.015708 and
            0.0221239 <= result[3] <= 3.4292 and
            0.1106200 <= result[4] <= 5.64159):
        raise ValueError
    return result


def _joints_positions_to_cartesian(ori, joint_1, joint_2, joint_3, joint_4, joint_5):
    u"""
    Прямая задача кинематики.

    Переводит обобщенные координты в декартовы
    """
    joint1_correction = radians(169) - 0.0100693
    joint2_correction = radians(65) - 0.0100693
    joint3_correction = radians(-146) + 0.015708
    joint4_correction = radians(102.5) - 0.0221239
    joint1_candle = joint_1 - joint1_correction
    joint2_candle = joint_2 - joint2_correction
    joint3_candle = joint_3 - joint3_correction
    joint4_candle = joint_4 - joint4_correction
    x_calc = cos(joint1_candle) * (217.5 * sin(joint2_candle + joint3_candle + joint4_candle) + 135 * sin(joint2_candle + joint3_candle) + 155 * sin(joint2_candle) + 33)
    y_calc = -sin(joint1_candle) * (217.5 * sin(joint2_candle + joint3_candle + joint4_candle) + 135 * sin(joint2_candle + joint3_candle) + 155 * sin(joint2_candle) + 33)
    z_calc = 217.5 * cos(joint2_candle + joint3_candle + joint4_candle) + 135 * cos(joint2_candle + joint3_candle) + 155 * cos(joint2_candle) + 147
    w_calc = joint2_candle + joint3_candle + joint4_candle
    if w_calc > pi:
        w_calc = w_calc - 2 * pi
    elif w_calc < - pi:
        w_calc = w_calc + 2 * pi
    else:
        w_calc = w_calc

    if ori == 0:
        w_calc = w_calc
    else:
        w_calc = - w_calc

    x_calc = round(x_calc, 1)
    y_calc = round(y_calc, 1)
    z_calc = round(z_calc, 1)
    w_calc = round(w_calc, 1)
    o_calc = round(joint_5, 1)

    joint_1d = round(degrees(joint1_candle), 4)
    joint_2d = round(degrees(joint2_candle), 4)
    joint_3d = round(degrees(joint3_candle), 4)
    joint_4d = round(degrees(joint4_candle), 4)
    print joint_1d, joint_2d, joint_3d, joint_4d
    print x_calc, y_calc, z_calc, w_calc

    return x_calc, y_calc, z_calc, w_calc, o_calc


def _calculate_angular_velocity(current, goal):
    u"""Calculate angular velocity to change current orientation to goal."""
    if current > goal:
        return -1
    elif current < goal:
        return 1
    else:
        return 0


def _calculate_velocity(*args):  # TODO: remove *args
    """Return velocity vector."""
    # TODO: Исправить вычисление скорости, чтобы робот не ездил по диагонали
    velocity = ()
    try:
        multiplier = 1 / sqrt(pow(args[0], 2) + pow(args[1], 2))
    except ZeroDivisionError:
        multiplier = 0
    velocity += (args[0] * multiplier),
    velocity += (args[1] * multiplier),
    return velocity


def _transform_coordinates(xwp, ywp, xwr, ywr, phir):
    u"""Transform point in world coordinates system into robot's system."""
    xwp -= xwr
    ywp -= ywr
    xyp = xwp * cos(phir) + ywp * sin(phir)
    yyp = -1 * xwp * sin(phir) + ywp * cos(phir)
    return xyp, yyp


def _check_pose(q, x, y, z, w, ori):
    u"""Проверяет заданное положение манипулятора."""

    x = round(x, 1)
    y = round(y, 1)
    z = round(z, 1)
    w = round(w, 1)
    axis_ok = (0.0100692 <= q[0] <= 5.84014 and
               0.0100692 <= q[1] <= 2.61799 and
               -5.0221239 <= q[2] <= -0.015708 and
               0.0221239 <= q[3] <= 3.4292 and
               0.1106200 <= q[4] <= 5.64159)
    platform_ok = not(z < 30 and x < 150 and -150 < y < 150)
    kinematic_ok = (x, y, z, w) == _joints_positions_to_cartesian(ori, *q)
    if (axis_ok and platform_ok and kinematic_ok):

        return True
    else:
        return False
