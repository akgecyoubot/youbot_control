#!/usr/bin/python2
# coding=UTF-8
"""This module is for testing rospyoubot module."""
PKG = 'pybotserver'

import rospyoubot
from math import radians
import unittest

class BaseTestCase(unittest.TestCase):
    def test_calculate_velocity(self):
        func = rospyoubot.calculate_velocity
        cases = {(0, 0): (0, 0),
                 (5, 0): (1, 0),
                 (0, 5): (0, 1),
                 (5, 5): (1, 1),
                 (-5, 0): (-1, 0),
                 (0, -5): (0, -1),
                 (-5, -5): (-1, -1),
                 (0.1, 0): (0.1, 0),
                 (0, 0.1): (0, 0.1),
                 (0.1, 0.1): (0.1, 0.1)}
        for test in cases.keys():
            self.assertEqual(func(*test),
                             cases[test],
                             'Wrong velocity calculated.')

    def test_transform_coordinates(self):
        func = rospyoubot.transform_coordinates
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
                            (5, 5, 3, 2, radians(35)): (1.31, -3.359),
                            (5, 5, 3, 2, radians(120)): (-3.232, -1.598),
                            (5, 5, 3, 2, radians(190)): (-2.135, 2.905),
                            (5, 5, 3, 2, radians(310)): (3.46, 1.013)}
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
