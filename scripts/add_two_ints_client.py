#!/usr/bin/env python
# coding=UTF-8

import sys
import rospy
from pybotserver.srv import *


def add_two_ints_client(x, y):
    u"""Основная функция клиента."""
    # Дожидаемся доступности сервиса add_two_ints
    rospy.wait_for_service('add_two_ints')
    try:
        # Создаём обработчик событий
        # Так как мы объявили тип AddTwoInts, то автоматически создастся
        # объект AddTwoIntsRequest
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    u"""Печатает на экран руководство по использованию."""
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s" % (x, y)
    print "%s + %s = %s" % (x, y, add_two_ints_client(x, y))
