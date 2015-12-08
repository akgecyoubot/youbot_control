#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(scan):
    for rng in scan.ranges:
        if scan.range_min < rng < DISTANCE:
            ROBOT.Base.set_velocity(0, 0, 0)
            break
        else:
            ROBOT.Base.set_velocity(VEL, 0, 0)


if __name__ == '__main__':
    ROBOT = rospyoubot.YouBot()
    DISTANCE = 0.4  # m
    VEL = 0.2  # m/s
    rospy.Subscriber("/scan", LaserScan, callback)
    # rospy.spin()
