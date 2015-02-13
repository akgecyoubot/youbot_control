#!/usr/bin/env python
# coding=UTF-8
u"""Node контролирующий youBot с джойстика."""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from brics_actuator.msg import JointValue, JointPositions, JointVelocities


class JoypadControlNode(object):

    u"""Класс Нода."""

    def __init__(self):
        u"""Функция инициализации."""
        self.base_velocity = Twist()
        self.gripper_position = JointPositions()
        self.arm_velocities = JointVelocities()
        self.base_velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        self.gripper_position_publisher = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
        self.arm_velocities_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities)
        rospy.Subscriber("/joy", Joy, self.update_data_from_joy)

    def update_data_from_joy(self, data):
        u""" Обновляет скорость при поступлении данных с джойстика.

        Запускается каждый раз при получении данных с джойстика
        Задаёт линейную скорость по оси X с 2 или 6 оси джойстика
        Задаёт линейную скорость по оси Y с 1 оси джойстика
        Задаёт угловую скорость вокруг оси Z с 5 оси джойстика
        """
        # Shutdown handling
        if data.buttons[10] or data.buttons[9]:
            rospy.signal_shutdown(u"До свидиния!")
        # Base velocity handling
        if abs(data.axes[1]) > abs(data.axes[5]):
            self.base_velocity.linear.x = data.axes[1] / 3.0
        else:
            self.base_velocity.linear.x = data.axes[5] / 3.0
        self.base_velocity.linear.y = data.axes[0] / 3.0
        self.base_velocity.angular.z = data.axes[4] / 3.0
        # Arm velocities handling
        self.gripper_position.positions = []
        # Open gripper
        if data.buttons[6]:
            A = JointValue()
            A.joint_uri = 'gripper_finger_joint_r'
            A.value = 0.011499
            A.unit = 'm'
            A.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(A)
            B = JointValue()
            B.joint_uri = 'gripper_finger_joint_l'
            B.value = 0.011499
            B.unit = 'm'
            B.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(B)
        # Close gripper
        if data.buttons[7]:
            A = JointValue()
            A.joint_uri = 'gripper_finger_joint_r'
            A.value = 0
            A.unit = 'm'
            A.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(A)
            B = JointValue()
            B.joint_uri = 'gripper_finger_joint_l'
            B.value = 0
            B.unit = 'm'
            B.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(B)
        # Joint 1 movement
        self.arm_velocities.velocities = []
        if notXOR(data.buttons[4], data.buttons[5]):
            A = JointValue()
            A.joint_uri = 'arm_joint_1'
            A.unit = 's^-1 rad'
            A.value = 0
            self.arm_velocities.velocities.append(A)
        elif data.buttons[4]:
            A = JointValue()
            A.joint_uri = 'arm_joint_1'
            A.unit = 's^-1 rad'
            A.value = 0.5
            self.arm_velocities.velocities.append(A)
        elif data.buttons[5]:
            A = JointValue()
            A.joint_uri = 'arm_joint_1'
            A.unit = 's^-1 rad'
            A.value = -0.5
            self.arm_velocities.velocities.append(A)

    def run(self):
        u"""Запускает Node и публикует сообщения по топику cmd_vel."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.base_velocity_publisher.publish(self.base_velocity)
            if self.gripper_position.positions:
                self.gripper_position_publisher.publish(self.gripper_position)
            if self.arm_velocities.velocities:
                self.arm_velocities_publisher.publish(self.arm_velocities)
            rate.sleep()


def notXOR(A, B):
    """Return True if only one of the inputs is True."""
    return (not A and not B) or (A and B)


if __name__ == '__main__':
    rospy.init_node('Joypad_controller')
    N = JoypadControlNode()
    N.run()
