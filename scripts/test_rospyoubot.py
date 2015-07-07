#!/usr/bin/python2
# coding=UTF-8
"""This module is for testing rospyoubot module."""

import rospyoubot
from math import radians, sqrt
import unittest

class BaseTestCase(unittest.TestCase):
    def test_calculateVelocity(self):
        func = rospyoubot._calculate_velocity
        cases = {(0, 0): (0, 0),
                 (5, 0): (1, 0),
                 (0, 5): (0, 1),
                 (-5, 0): (-1, 0),
                 (0, -5): (0, -1)}
        for test in cases.keys():
            real = func(*test)
            expected = cases[test]
            self.assertEqual(real,
                             expected)
            if expected != (0, 0):
                self.assertEqual(sqrt(pow(real[0], 2) + pow(real[1], 2)), 1)

    def test_transformCoordinates(self):
        func = rospyoubot._transform_coordinates
        cases = {(0, 0, 0, 0, 0): (0, 0),
                 (1, 0, 0, 0, 0): (1, 0),
                 (0, 1, 0, 0, 0): (0, 1),
                 (1, 1, 0, 0, 0): (1, 1),
                 (-1, 0, 0, 0, 0): (-1, 0),
                 (0, -1, 0, 0, 0): (0, -1),
                 (-1, -1, 0, 0, 0): (-1, -1),
                 (0, 0, 1, 0, 0): (-1, 0),
                 (0, 0, 0, 1, 0): (0, -1),
                 (0, 0, 1, 1, 0): (-1, -1)}
        almostEqualCases = {(0, 0, 0, 0, radians(45)): (0, 0),
                            (5, 5, 0, 0, radians(30)): (6.83, 1.83),
                            (5, 5, 0, 0, radians(70)): (6.41, -2.99),
                            (5, 5, 0, 0, radians(190)): (-5.79, -4.06),
                            (5, 5, 0, 0, radians(245)): (-6.64, 2.42),
                            (7, -8, -2, 3, radians(25)): (3.51, -13.77),
                            (7, -8, -2, 3, radians(110)): (-13.41, -4.7),
                            (7, -8, -2, 3, radians(150)): (-13.29, 5.03),
                            (7, -8, -2, 3, radians(230)): (2.64, 13.97),
                            (7, -8, 4, -3, radians(20)): (1.11, -5.72),
                            (7, -8, 4, -3, radians(65)): (-3.26, -4.83),
                            (7, -8, 4, -3, radians(155)): (-4.83, 3.26),
                            (7, -8, 4, -3, radians(-108)): (3.83, 4.4)}
        for test in cases.keys():
            self.assertEqual(func(*test),
                             cases[test])
        for test in almostEqualCases.keys():
            real = func(*test)
            expected = almostEqualCases[test]
            self.assertAlmostEqual(real[0],
                                   expected[0],
                                   places=2)
            self.assertAlmostEqual(real[1],
                                   expected[1],
                                   places=2)
    def test_calculateAngularVelocity(self):
        func = rospyoubot._calculate_angular_velocity
        cases = {(0, 0): (0),
                 (0, 1): (1),
                 (1, 0): (-1),
                 (1, 1): (0),
                 (0, -1): (-1),
                 (-1, 0): (1)}
        for test in cases.keys():
            self.assertEqual(func(*test),
                             cases[test],
                             "Function({}) is not equal to {}".format(test, cases[test]))
class ArmTestCase(unittest.TestCase):
    def test_jointsAnglesForPose(self):
        pass

    def test_checkPose(self):
        pass
