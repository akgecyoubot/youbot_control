#!/usr/bin/python2
# coding=UTF-8
"""This module is for testing rospyoubot module."""

import rospyoubot
from math import radians, sin, cos

class TestBase:
    def test_calculate_velocity(self):
        assert rospyoubot.calculate_velocity(0, 0) == (0, 0)
        assert rospyoubot.calculate_velocity(5, 0) == (1, 0)
        assert rospyoubot.calculate_velocity(0, 5) == (0, 1)
        assert rospyoubot.calculate_velocity(5, 5) == (1, 1)
        assert rospyoubot.calculate_velocity(-5, 0) == (-1, 0)
        assert rospyoubot.calculate_velocity(0, -5) == (0, -1)
        assert rospyoubot.calculate_velocity(-5, -5) == (-1, -1)
        assert rospyoubot.calculate_velocity(0.1, 0) == (0.1, 0)
        assert rospyoubot.calculate_velocity(0, 0.1) == (0, 0.1)
        assert rospyoubot.calculate_velocity(0.1, 0.1) == (0.1, 0.1)

    def test_transform_coordinates(self):
        assert rospyoubot.transform_coordinates(0, 0, 0, 0, 0) == (0, 0)
        assert rospyoubot.transform_coordinates(1, 0, 0, 0, 0) == (1, 0)
        assert rospyoubot.transform_coordinates(0, 1, 0, 0, 0) == (0, 1)
        assert rospyoubot.transform_coordinates(1, 1, 0, 0, 0) == (1, 1)
        assert rospyoubot.transform_coordinates(-1, 0, 0, 0, 0) == (-1, 0)
        assert rospyoubot.transform_coordinates(0, -1, 0, 0, 0) == (0, -1)
        assert rospyoubot.transform_coordinates(-1, -1, 0, 0, 0) == (-1, -1)
        assert rospyoubot.transform_coordinates(0, 0, 1, 0, 0) == (-1, 0)
        assert rospyoubot.transform_coordinates(0, 0, 0, 1, 0) == (0, -1)
        assert rospyoubot.transform_coordinates(0, 0, 1, 1, 0) == (-1, -1)
        assert rospyoubot.transform_coordinates(0, 0, 0, 0, radians(45)) == (0, 0)
        # assert rospyoubot.transform_coordinates(1, 1, 0, 0, radians(45)) == (1 * sin(radians(45)), 1 * cos(radians(45)))

def tst_robot():
    import rospy
    """Test rospyoubot functionality."""
    robot = rospyoubot.YouBot()
    timer = rospy.Rate(0.25)
    timer.sleep()
    print "Testing base velocities command..."
    robot.base.set_velocity(0.1, 0.1, -0.1)
    timer.sleep()
    robot.base.set_velocity()
    timer.sleep()
    print "Testing gripper position command..."
    robot.arm.gripper.set_gripper_state(False)
    timer.sleep()
    robot.arm.gripper.set_gripper_state(True)
    timer.sleep()
    print "Testing arm position command..."
    robot.arm.set_joints_angles(2.95, 1.1, -2.6, 1.8, 2.95)
    timer.sleep()
    robot.arm.set_joints_angles(0.0100693,
                                0.0100693,
                                -0.015708,
                                0.0221239,
                                0.11062)
    timer.sleep()
    print "Testing arm velocities commad..."
    robot.arm.set_joints_velocities(0.1, 0.1, -0.1, 0.1, 0.1)
    timer.sleep()
    robot.arm.set_joints_velocities(0, 0, 0, 0, 0)
    timer.sleep()
    print "Going home..."
    robot.arm.set_joints_angles(0.0100693,
                                0.0100693,
                                -0.015708,
                                0.0221239,
                                0.11062)
