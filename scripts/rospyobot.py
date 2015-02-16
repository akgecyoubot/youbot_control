#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for KUKA youBot ROS interface."""

import rospy


class YouBot(object):

    """youBot class.

    Has Base and Arm atributes.
    """

    def __init__(self):
        """Class constructor."""
        self.arm = Arm()
        self.base = Base()

    def run(self):
        pass

    def calibrate(self):
        pass

class Base(object):

    """Control youBot Base."""

    def set_velocity(self):
        pass

    def get_odometry(self):
        pass

class Arm(object):

    """Control youBot Arm."""

    def set_joints_angles(self):
        pass

    def set_joints_velocity(self):
        pass

    def get_joints_angles(self):
        pass

    def toggle_gripper_state(self):
        pass
