#!/usr/bin/env python
# coding=UTF-8
u"""Node контролирующий youBot с джойстика."""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoypadControlNode(object):

    u"""Класс Нода."""

    def __init__(self):
        u"""Функция инициализации."""
        self.velocity = Twist()
        rospy.init_node('Joypad_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber("joy", Joy, self.update_data)

    def convert_joy_to_velocity(self, data):
        u"""Преобразует информацию с джойстика и возвращает вектор скорости."""
        if data.buttons[10] or data.buttons[9]:
            rospy.signal_shutdown(u"До свидиния!")
        velocity = Twist()
        velocity.linear.x = data.axes[1] / 4.0
        velocity.linear.y = data.axes[0] / 4.0
        velocity.angular.x = data.axes[3]
        velocity.angular.z = data.axes[2]
        return velocity

    def update_data(self, data):
        u"""Обновляет скорость при поступлении данных с джойстика."""
        self.velocity = self.convert_joy_to_velocity(data)

    def run(self):
        u"""Публикует сообщения по топику cmd_vel."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(N.velocity)
            rate.sleep()


if __name__ == '__main__':
    N = JoypadControlNode()
    N.run()
