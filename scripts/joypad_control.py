#!/usr/bin/env python
# coding=UTF-8
u"""TODO: add module documentation."""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from brics_actuator.msg import JointValue, JointPositions, JointVelocities


class JoypadControlNode(object):
    # pylint: disable=R0902

    u"""TODO: Add class documentation."""

    def __init__(self):
        u"""Конструктор класса."""
        self.base_velocity = Twist()
        self.gripper_position = JointPositions()
        self.arm_velocities = JointVelocities()
        self.base_speed_multiplier = 1
        self.arm_speed_multiplier = 0.33
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
        # Close gripper
        if data.buttons[7]:
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

    def update_arm_velocities(self, data):
        u"""Задаёт скорости осей манипулятора."""
        self.arm_velocities.velocities = []
        if notXOR(data.buttons[4], data.buttons[5]):
            tmp_joint_value = JointValue()
            tmp_joint_value.joint_uri = 'arm_joint_1'
            tmp_joint_value.unit = 's^-1 rad'
            tmp_joint_value.value = 0
            self.arm_velocities.velocities.append(tmp_joint_value)
        elif data.buttons[4]:
            tmp_joint_value = JointValue()
            tmp_joint_value.joint_uri = 'arm_joint_1'
            tmp_joint_value.unit = 's^-1 rad'
            tmp_joint_value.value = 1 * self.arm_speed_multiplier
            self.arm_velocities.velocities.append(tmp_joint_value)
        elif data.buttons[5]:
            tmp_joint_value = JointValue()
            tmp_joint_value.joint_uri = 'arm_joint_1'
            tmp_joint_value.unit = 's^-1 rad'
            tmp_joint_value.value = -1 * self.arm_speed_multiplier
            self.arm_velocities.velocities.append(tmp_joint_value)

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
