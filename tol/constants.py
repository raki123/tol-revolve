"""
ToL constants
"""
from __future__ import absolute_import
import math
from revolve.build.sdf import PID

MAX_HIDDEN_NEURONS = 10
""" Maximum number of hidden neurons """

MAX_SERVO_TORQUE = (1.8 * 9.81) / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

MAX_SERVO_TORQUE_ROTATIONAL = (4 * 9.81) / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

SERVO_LIMIT = math.radians(45)
""" Upper and lower limit """

SERVO_PID = PID(proportional_gain=1.0, integral_gain=0.1)
""" Default servo PID """
