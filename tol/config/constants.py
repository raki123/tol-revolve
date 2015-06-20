"""
ToL constants
"""
from __future__ import absolute_import
import math
from revolve.build.sdf import PID

MAX_HIDDEN_NEURONS = 10
""" Maximum number of hidden neurons """

MAX_SERVO_TORQUE = 0.1 * (1.8 * 9.81) / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

MAX_SERVO_TORQUE_ROTATIONAL = 0.1 * (4 * 9.81) / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

MAX_SERVO_VELOCITY = -(50.0/60.0) * 2 * math.pi
""" Maximum rotational velocity of a servo, in radians / second """

SERVO_LIMIT = math.radians(45)
""" Upper and lower limit """

CARDAN_LIMIT = math.radians(45)
""" Upper and lower limit of each axis of rotation """

SERVO_PID = PID(proportional_gain=0.1, integral_gain=0.01)
""" Default servo PID """
