#!/usr/bin/env python
"""Hokuyo Laser Scanner demo programm."""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(scan):
    """Get LaserScan data and move youBot base if there's no obstacles ahead."""
    vel = Twist()
    for rng in scan.ranges:
        if scan.range_min < rng < 0.4:
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            VEL_PUB.publish(vel)
            break
    else:
        vel.linear.x = 0.1
        vel.linear.y = 0
        vel.angular.z = 0
        VEL_PUB.publish(vel)


if __name__ == '__main__':
    VEL_PUB = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('Hokuyo_demo', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
