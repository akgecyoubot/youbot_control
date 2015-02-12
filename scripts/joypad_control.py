#!/usr/bin/env python
# coding=UTF-8
u"""Node контролирующий youBot с джойстика."""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from brics_actuator.msg import JointVelocities, JointValue


class JoypadControlNode(object):

    u"""Класс Нода."""

    def __init__(self):
        u"""Функция инициализации."""
        self.base_velocity = Twist()
        self.arm_velocities = JointVelocities()
        self.base_velocity_publisher = rospy.Publisher('cmd_vel', Twist)
        self.arm_velocities_publisher = rospy.Publisher('velocity_command',
                                                        JointVelocities)
        rospy.Subscriber("joy", Joy, self.update_data_from_joy)

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
        a = JointValue()
        self.arm_velocities.velocities.append(a)
        self.arm_velocities.velocities[0].joint_uri = 'arm_joint_1'
        # self.arm_velocities.velocities.unit = 's^-1 rad'
        self.arm_velocities.velocities[0].value = data.axes[2] / 3.0

    def run(self):
        u"""Запускает Node и публикует сообщения по топику cmd_vel."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.base_velocity_publisher.publish(self.base_velocity)
            self.arm_velocities_publisher.publish(self.arm_velocities)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Joypad_controller')
    N = JoypadControlNode()
    N.run()
