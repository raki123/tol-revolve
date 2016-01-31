"""
ToL constants
"""
from __future__ import absolute_import
import math
from revolve.build.sdf import PID

MAX_HIDDEN_NEURONS = 10
""" Maximum number of hidden neurons """

MAX_SERVO_TORQUE = 1.8 * 9.81 / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

MAX_SERVO_TORQUE_ROTATIONAL = 4 * 9.81 / 100
""" Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100 """

MAX_SERVO_VELOCITY = (50.0/60.0) * 2 * math.pi
""" Maximum rotational velocity of a servo, in radians / second """

SERVO_LIMIT = math.radians(45)
""" Upper and lower limit """

HINGE_LIMIT = math.radians(45)
""" Upper and lower limit for hinge """

CARDAN_LIMIT = math.radians(45)
""" Upper and lower limit of each axis of rotation """

SERVO_VELOCITY_PID = PID(
    proportional_gain=0.5,
    derivative_gain=0.0,
    integral_gain=0,
    integral_max=0
)
""" Default servo velocity PID. **Currently unused** as velocity is abstractly set
    on the servo. """

SERVO_POSITION_PID = PID(
    proportional_gain=0.9,
    derivative_gain=0,
    integral_gain=0,
    integral_max=0
)
""" Default servo position PID. """

# Original parameter values from RobogenCollision.cpp
SURFACE_FRICTION1 = 1.0
SURFACE_FRICTION2 = 1.0
SURFACE_SLIP1 = 0.01
SURFACE_SLIP2 = 0.01

# I calculated these parameters to match the soft_erp / soft_cfm
# values used in Robogen (they cannot be set directly in Gazebo
# it seems). The division in `kp` is the intended time step length.
SURFACE_KD = 4
SURFACE_KP = 96 / 0.005

# Thickness and height of the arena walls in meters
WALL_THICKNESS = 0.05
WALL_HEIGHT = 1.0
