#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for KUKA youBot ROS interface."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class YouBot(object):

    """youBot class.

    Has Base and Arm atributes.
    """

    def __init__(self):
        """Class constructor."""
        self.arm = Arm()
        self.base = Base()
        self.freq = 10
        rospy.init_node('rospyoubot')
        self.base_velocity_publisher = rospy.Publisher('/cmd_vel', Twist)

    def publish(self):
        self.base.publish()

class Base(object):

    """Control youBot Base."""

    def __init__(self):
        """Class constructor."""
        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.linear.y = 0
        self.velocity.angular.z = 0
        self.odometry = Odometry()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

    def publish(self):
        self.velocity_publisher.publish(self.velocity)

    def set_velocity(self, lin_x=0, lin_y=0, ang_z=0):
        """Set base velocity vector."""
        self.velocity.linear.x = lin_x
        self.velocity.linear.y = lin_y
        self.velocity.angular.z = ang_z

    def update_odometry(self, data):
        self.odometry = data

    def get_odometry(self):
        return self.odometry.pose.covariance

class Arm(object):

    """Control youBot Arm."""

    def __init__(self):
        """Class constructor."""
