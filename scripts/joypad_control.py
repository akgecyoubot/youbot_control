#!/usr/bin/env python
# coding=UTF-8
u"""Node контролирующий youBot с джойстика."""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from brics_actuator.msg import JointValue, JointPositions, JointVelocities


class JoypadControlNode(object):
    # pylint: disable-msg=R0902

    u"""Класс Нода."""

    def __init__(self):
        u"""Функция инициализации."""
        self.base_velocity = Twist()
        self.gripper_position = JointPositions()
        self.arm_velocities = JointVelocities()
        self.base_speed_multiplier = 0.33
        self.arm_speed_multiplier = 1.0
        self.base_velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        self.gripper_position_publisher = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
        self.arm_velocities_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities)
        rospy.Subscriber("/joy", Joy, self.update_data_from_joy)

    def update_data_from_joy(self, data):
        u""" Обновляет данные при поступлении информации с джойстика.

        Запускается каждый раз при получении данных с джойстика
        """
        # Shutdown handling
        if data.buttons[10] or data.buttons[11]:
            rospy.signal_shutdown(u"До свидиния!")
        self.update_base_velocity(data)
        self.update_gripper_position(data)
        self.update_arm_velocities(data)

    def update_base_velocity(self, data):
        u"""Задаёт скорость базы."""
        if abs(data.axes[1]) > abs(data.axes[5]):
            self.base_velocity.linear.x = data.axes[1] * self.base_speed_multiplier
        else:
            self.base_velocity.linear.x = data.axes[5] * self.base_speed_multiplier
        self.base_velocity.linear.y = data.axes[0] * self.base_speed_multiplier
        self.base_velocity.angular.z = data.axes[4] * self.base_speed_multiplier

    def update_gripper_position(self, data):
        u"""Открывает/закрывает гриппер."""
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

    def update_arm_velocities(self, data):
        u"""Задаёт скорости осей манипулятора."""
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
            A.value = 1 * self.arm_speed_multiplier
            self.arm_velocities.velocities.append(A)
        elif data.buttons[5]:
            A = JointValue()
            A.joint_uri = 'arm_joint_1'
            A.unit = 's^-1 rad'
            A.value = -1 * self.arm_speed_multiplier
            self.arm_velocities.velocities.append(A)

    def run(self):
        u"""Запускает Node и публикует сообщения по топикам."""
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
