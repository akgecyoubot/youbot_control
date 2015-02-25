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
        self.arm = YouBot.Arm()
        self.base = YouBot.Base()

    class Base(object):

        """Control youBot Base.

        TODO: write documentation for this class
        """

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
                tuple: (X, Y, Z, rotation X, rotation Y, rotation Z)

            Возвращает одометрию.

            Возвращает:
                кортеж (X, Y, Z, поворот по X, поворот по Y, поворот по Z)
            """
            position = ()
            position += self.odometry.pose.pose.position.x,
            position += self.odometry.pose.pose.position.y,
            position += self.odometry.pose.pose.orientation.z,
            return position

    class Arm(object):

        """Control youBot Arm.

        TODO: write documentation for this class
        """

        def __init__(self):
            u"""Class constructor.

            Конструктор класса.
            """
            self.gripper = YouBot.Gripper()
            self.joints_positions = JointPositions()
            self.current_joints_states = JointState()
            for i in range(5):
                self.current_joints_states.position.append(0.0)
            self.joints_velocities = JointVelocities()
            self.joints_positions_publisher = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions)
            self.joints_velocities_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities)
            rospy.Subscriber('/joint_states', JointState, self._update_joints_states)

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
            for i in range(len(args)):  # I know it's unpythonic, sorry
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

            Устанавливает скорость каждоый степени подвижности в радианах/с.

            Аргументы:
                *args -- скорости соотвествующих степеней (j1, j2, j3, j4, j5)
            """
            assert len(args) <= 5
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
            """TODO: write documentation for this function."""
            return self.current_joints_states.position

    class Gripper(object):

        """Gripper class.

        TODO: write documentation for this class
        """

        def __init__(self):
            """Gripper constructor."""
            self.gripper_position = JointPositions()
            self.gripper_position_publisher = rospy.Publisher('arm_1/gripper_controller/position_command', JointPositions)

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