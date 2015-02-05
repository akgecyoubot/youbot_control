#!/usr/bin/env python
# coding=UTF-8
"""
Simple talker demo.

Listens to std_msgs/Strings published
to the 'chatter' topic
"""

import rospy
from std_msgs.msg import String


def callback(data):
    u"""Функция вызываемая по приходу сообщения."""
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    u"""Функция прослушивания топика."""
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    # Подписываемся на топик chatter типа String, при получении сообщения
    # вызываем функцию callback() с сообщение в качестве первого аргумента
    rospy.spin()
    # rospy.spin() предотвращает завершение Node

if __name__ == '__main__':
    listener()
