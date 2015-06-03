"""
ToL constants
"""
from __future__ import absolute_import
import math

MAX_HIDDEN_NEURONS = 10
""" Maximum number of hidden neurons """

MAX_SERVO_TORQUE = (1.8 * 9.81) / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

SERVO_LIMIT = math.radians(45)
""" Upper and lower limit """
