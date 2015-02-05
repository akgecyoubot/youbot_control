#!/usr/bin/env python
# coding=UTF-8
u"""Пример реализации Publisher Node на rospy."""

import rospy
from std_msgs.msg import String


def talker():
    u"""Функция публикующая сообщения."""
    # Создаём нового Publisher'а с топиком chatter типа String
    pub = rospy.Publisher('chatter', String)
    # Инициализируем Node с именем talker
    rospy.init_node('talker', anonymous=True)
    # Устанавливаем частоту обновления в Герцах
    rate = rospy.Rate(1)
    # Пока Node работает:
    while not rospy.is_shutdown():
        # Формируем сообщение
        hello_str = "hello world %s" % rospy.get_time()
        # Отправляем сообщение в лог
        rospy.loginfo(hello_str)
        # Публикуем наше сообщение
        pub.publish(hello_str)
        # Вызываем функцию sleep() чтобы выдержать нашу частоту обовления
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
