#!/usr/bin/python2
# coding=UTF-8
"""This module is for testing rospyoubot module."""

import rospyoubot
import rospy

def test():
    R1 = rospyoubot.YouBot()
    timer = rospy.Rate(0.25)
    timer.sleep()
    print "Testing base velocities command..."
    R1.base.set_velocity(0.1, 0.1, -0.1)
    timer.sleep()
    R1.base.set_velocity()
    timer.sleep()
    print "Testing gripper position command..."
    R1.arm.gripper.set_gripper_state(False)
    timer.sleep()
    R1.arm.gripper.set_gripper_state(True)
    timer.sleep()
    print "Testing arm position command..."
    R1.arm.set_joints_angles(2.95, 1.1, -2.6, 1.8, 2.95)
    timer.sleep()
    R1.arm.set_joints_angles(0.0100693, 0.0100693, -0.015708, 0.0221239, 0.11062)
    timer.sleep()
    print "Testing arm velocities commad..."
    R1.arm.set_joints_velocities(0.1, 0.1, -0.1, 0.1, 0.1)
    timer.sleep()
    R1.arm.set_joints_velocities(0, 0, 0, 0, 0)
    timer.sleep()
    print "Going home..."
    R1.arm.set_joints_angles(0.0100693, 0.0100693, -0.015708, 0.0221239, 0.11062)


if __name__ == '__main__':
    test()
