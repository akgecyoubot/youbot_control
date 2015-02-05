#!/usr/bin/env python
# coding=UTF-8

import rospy
from pybotserver.srv import AddTwoInts


def handle_add_two_ints(req):
    u"""Печатает на экран возвращаемый ответ и возвращает сумму."""
    print "Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b))
    # return AddTwoIntsResponse(req.a + req.b)
    return req.a + req.b


def add_two_ints_server():
    u"""Создаёт нод и обрабатывает запрос."""
    # Декларируем Node
    rospy.init_node('add_two_ints_server')
    # Декларируем наш Сервис типа AddTwoInts
    # И перенаправляем запросы функцие handle_add_two_ints
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
