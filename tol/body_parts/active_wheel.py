# Revolve imports
from revolve.build.sdf import BodyPart, VelocityMotor, PID
from revolve.build.util import in_grams, in_mm

from sdfbuilder.joint import Joint, Limit
from sdfbuilder.math import Vector3

# Local imports
from .util import ColorMixin
from .. import constants

MASS_SLOT = in_grams(4)
MASS_SERVO = in_grams(9)
MASS_WHEEL = in_grams(5)
SLOT_WIDTH = in_mm(34)
SLOT_THICKNESS = in_mm(1.5)
SERVO_WIDTH = in_mm(14)
WHEEL_THICKNESS = in_mm(3)
SERVO_LENGTH = in_mm(36.8) - WHEEL_THICKNESS
SERVO_HEIGHT = in_mm(14)
SEPARATION = in_mm(1.0)
X_SERVO = -SLOT_THICKNESS + SEPARATION + 0.5 * SERVO_LENGTH
X_WHEEL = X_SERVO + 0.5 * SERVO_LENGTH

class ActiveWheel(BodyPart, ColorMixin):
    """
    Active wheel
    """

    def _initialize(self, **kwargs):
        self.radius = in_mm(kwargs['radius'])

        wheel = self.create_link("wheel")
        servo = self.create_link("servo")
        self.root = self.create_link("wheel_root")

        # Create the root
        self.root.make_box(MASS_SLOT, SLOT_WIDTH,
                           SLOT_WIDTH, SLOT_THICKNESS)

        # Create the servo
        servo.make_box(MASS_SERVO, SERVO_HEIGHT, SERVO_WIDTH, SERVO_LENGTH)
        servo.set_position(Vector3(0, 0, X_SERVO))

        # Create the wheel
        wheel.make_cylinder(MASS_WHEEL, self.radius, WHEEL_THICKNESS)
        wheel.set_position(Vector3(0, 0, X_WHEEL))

        # Fix the root to the servo
        self.fix_links(self.root, servo, Vector3(0, 0, -0.5 * SERVO_LENGTH), Vector3(0, 0, 1))

        # Attach the wheel and the root with a revolute joint
        self.joint = Joint("revolute", servo, wheel, axis=Vector3(0, 0, -1))
        self.joint.set_position(Vector3(0, 0, -WHEEL_THICKNESS))
        self.joint.axis.limit = Limit(effort=constants.MAX_SERVO_TORQUE_ROTATIONAL)
        self.add_joint(self.joint)

        # Now we add a motor that powers the joint. This particular servo
        # targets a velocity. Use a simple PID controller initially.
        pid = PID(proportional_gain=1.0, integral_gain=0.1)
        self.motors.append(VelocityMotor(self.id, "rotate", self.joint, pid))

        # Call color mixin
        self.apply_color()

    def get_slot(self, slot_id):
        self.check_slot(slot_id)
        return self.root

    def get_slot_position(self, slot_id):
        return Vector3(0, 0, -0.5 * SLOT_THICKNESS)

    def get_slot_normal(self, slot_id):
        return Vector3(0, 0, -1)

    def get_slot_tangent(self, slot_id):
        return Vector3(0, 1, 0)
