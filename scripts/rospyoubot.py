#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for ROS wrapper for KUKA youBot API."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
from sensor_msgs.msg import JointState


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
                    velocity.append(d/abs(d))
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
            tmp.joint_uri = 'arm_joint_{}'.format(i+1)
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
            tmp.joint_uri = 'arm_joint_{}'.format(i+1)
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

def test():
    """Test rospyoubot functionality."""
    robot = YouBot()
    timer = rospy.Rate(0.25)
    timer.sleep()
    print "Testing base velocities command..."
    robot.base.set_velocity(0.1, 0.1, -0.1)
    timer.sleep()
    robot.base.set_velocity()
    timer.sleep()
    print "Testing gripper position command..."
    robot.arm.gripper.set_gripper_state(False)
    timer.sleep()
    robot.arm.gripper.set_gripper_state(True)
    timer.sleep()
    print "Testing arm position command..."
    robot.arm.set_joints_angles(2.95, 1.1, -2.6, 1.8, 2.95)
    timer.sleep()
    robot.arm.set_joints_angles(0.0100693,
                                0.0100693,
                                -0.015708,
                                0.0221239,
                                0.11062)
    timer.sleep()
    print "Testing arm velocities commad..."
    robot.arm.set_joints_velocities(0.1, 0.1, -0.1, 0.1, 0.1)
    timer.sleep()
    robot.arm.set_joints_velocities(0, 0, 0, 0, 0)
    timer.sleep()
    print "Going home..."
    robot.arm.set_joints_angles(0.0100693,
                                0.0100693,
                                -0.015708,
                                0.0221239,
                                0.11062)

def test_goto():
    r = YouBot()
    r.base.lin_goto(1, 0, 0)
    r.base.lin_goto(0, 0, 0)


if __name__ == '__main__':
    test_goto()
