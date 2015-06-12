# Revolve imports
import itertools
import math
from revolve.build.sdf import BodyPart, VelocityMotor, PID
from revolve.build.util import in_grams, in_mm

from sdfbuilder.joint import Joint, Limit
from sdfbuilder.math import Vector3

# Local imports
from .util import ColorMixin
from .. import constants

MASS_SLOT = in_grams(4)
MASS_SERVO = in_grams(9)
MASS_WHEG = in_grams(3)

SLOT_WIDTH = in_mm(34)
SLOT_THICKNESS = in_mm(1.5)
WHEG_BASE_RADIUS = in_mm(9)
WHEG_THICKNESS = in_mm(4)
WHEG_WIDTH = in_mm(4)

# z-center shift w.r.t. slot z-center
SERVO_Z_OFFSET = in_mm(9)
SERVO_WIDTH = in_mm(14)

# Take of wheg thickness because they overlap in reality
SERVO_LENGTH = in_mm(36.8) - WHEG_THICKNESS
SERVO_HEIGHT = in_mm(14)

SEPARATION = in_mm(1.0)
X_SERVO = 0.5 * SERVO_LENGTH - SLOT_THICKNESS + SEPARATION

X_WHEG_BASE = X_SERVO + 0.5 * (SERVO_LENGTH + WHEG_THICKNESS)

class ActiveWheg(BodyPart, ColorMixin):
    """
    Active wheel
    """

    def _initialize(self, **kwargs):
        self.radius = in_mm(kwargs['radius'])

        self.root = self.create_link("root")
        servo = self.create_link("servo")
        wheg_base = self.create_link("wheg_base")
        spoke1 = self.create_link("spoke1")
        spoke2 = self.create_link("spoke2")
        spoke3 = self.create_link("spoke3")

        # Because of the cylinder shapes, x axis is swapped with z axis
        # as compared to the Robogen code.
        # Initialize root
        self.root.make_box(MASS_SLOT, SLOT_WIDTH, SLOT_WIDTH, SLOT_THICKNESS)

        # Initialize servo
        z_servo = 0
        servo.make_box(MASS_SERVO, SERVO_HEIGHT, SERVO_WIDTH, SERVO_LENGTH)
        servo.set_position(Vector3(z_servo, 0, X_SERVO))

        # Initialize the base
        spoke_mass = MASS_WHEG / 4.0
        wheg_base_radius = WHEG_BASE_RADIUS * self.radius
        wheg_base.make_cylinder(spoke_mass, wheg_base_radius, WHEG_THICKNESS)
        wheg_base.set_position(Vector3(z_servo, 0, X_WHEG_BASE))

        # Initialize the spokes
        spoke1.make_box(spoke_mass, self.radius, WHEG_WIDTH, WHEG_THICKNESS)
        spoke2.make_box(spoke_mass, self.radius, WHEG_WIDTH, WHEG_THICKNESS)
        spoke3.make_box(spoke_mass, self.radius, WHEG_WIDTH, WHEG_THICKNESS)

        # Rotate the spokes
        spokes = [spoke1, spoke2, spoke3]
        """ :type : list[Posable] """
        rotations = [60, 180, 300]
        axis = Vector3(0, 0, 1)
        r = wheg_base_radius + 0.5 * self.radius

        for spoke, rotation in itertools.izip(spokes, rotations):
            spoke.rotate_around(axis, math.radians(rotation))
            rotate_radians = math.radians(rotation)
            x = r * math.cos(rotate_radians)
            y = r * math.sin(rotate_radians)
            spoke.set_position(Vector3(z_servo + x, y, X_WHEG_BASE))

        # Create the connecting joints
        self.fix_links(self.root, servo, Vector3(0, 0, -0.5 * SERVO_LENGTH), Vector3(0, 0, 1))

        for spoke in spokes:
            self.fix_links(wheg_base, spoke, Vector3(-0.5 * self.radius, 0, 0), Vector3(1, 0, 0))

        # Revolute joint of the servo
        self.joint = Joint("revolute", servo, wheg_base, axis=Vector3(0, 0, 1))
        self.joint.axis.limit = Limit(effort=constants.MAX_SERVO_TORQUE_ROTATIONAL)
        self.add_joint(self.joint)

        # Now we add a motor that powers the joint. This particular servo
        # targets a velocity. Use a simple PID controller initially.
        pid = constants.SERVO_PID
        self.motors.append(VelocityMotor(self.id, "rotate", self.joint, pid))

        # Call color mixin
        self.apply_color()

    def get_slot(self, slot_id):
        self.check_slot(slot_id)
        return self.root

    def get_slot_position(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(0, 0, -0.5 * SLOT_THICKNESS)

    def get_slot_normal(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(0, 0, -1)

    def get_slot_tangent(self, slot_id):
        return Vector3(0, 1, 0)
