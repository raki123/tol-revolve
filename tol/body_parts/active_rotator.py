# Revolve imports
from revolve.build.sdf import BodyPart, PositionMotor
from revolve.build.util import in_grams, in_mm

from sdfbuilder.joint import Joint, Limit
from sdfbuilder.math import Vector3

# Local imports
from .util import ColorMixin
from ..config import constants

MASS_SLOT = in_grams(4)
MASS_SERVO = in_grams(9)
MASS_CONNECTION_SLOT = in_grams(2)

SLOT_WIDTH = in_mm(34)
SLOT_THICKNESS = in_mm(1.5)
SERVO_Z_OFFSET = in_mm(0)
SERVO_WIDTH = in_mm(14)
SERVO_LENGTH = in_mm(36.8)
SERVO_HEIGHT = in_mm(14)
JOINT_CONNECTION_THICKNESS = in_mm(7.5)
JOINT_CONNECTION_WIDTH = in_mm(34)
SEPARATION = in_mm(0.1)

class ActiveRotator(BodyPart, ColorMixin):
    """
    Active wheel
    """

    def _initialize(self, **kwargs):
        self.root = self.create_link("root")
        servo = self.create_link("servo")
        self.connection = self.create_link("connection")

        # Initialize the root
        self.root.make_box(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH)

        # Initialize the servo
        x_servo = 0.5 * (SLOT_THICKNESS + SERVO_LENGTH) + SEPARATION
        # z_servo = 0.5 * (SERVO_HEIGHT - SLOT_WIDTH) + SERVO_Z_OFFSET
        z_servo = 0
        servo.make_box(MASS_SERVO, SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT)
        servo.set_position(Vector3(x_servo, 0, z_servo))

        # Initialize the connection
        x_conn = x_servo + 0.5 * (SERVO_LENGTH - JOINT_CONNECTION_THICKNESS) + SEPARATION
        self.connection.make_box(MASS_CONNECTION_SLOT, JOINT_CONNECTION_THICKNESS,
                                 JOINT_CONNECTION_WIDTH, JOINT_CONNECTION_WIDTH)
        self.connection.set_position(Vector3(x_conn, 0, 0))

        # Fix the links
        # root <-> servo
        self.fix_links(self.root, servo, Vector3(-0.5 * SERVO_LENGTH, 0, 0),
                       Vector3(1, 0, 0))

        # servo <revolute> connection
        self.joint = Joint("revolute", servo, self.connection, axis=Vector3(1, 0, 0))
        self.joint.axis.limit = Limit(effort=constants.MAX_SERVO_TORQUE_ROTATIONAL)
        self.add_joint(self.joint)

        # Now we add a motor that powers the joint. This particular servo
        # targets a velocity. Use a simple PID controller initially.
        pid = constants.SERVO_PID
        self.motors.append(PositionMotor(self.id, "rotate", self.joint, pid))

        # Call color mixin
        self.apply_color()

    def get_slot(self, slot_id):
        self.check_slot(slot_id)
        return self.root if slot_id == 0 else self.connection

    def get_slot_position(self, slot_id):
        self.check_slot(slot_id)
        if slot_id == 0:
            return Vector3(-0.5 * SLOT_THICKNESS, 0, 0)
        else:
            return self.connection.to_sibling_frame(
                Vector3(0.5 * JOINT_CONNECTION_THICKNESS, 0, 0), self)

    def get_slot_normal(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(-1, 0, 0) if slot_id == 0 else Vector3(1, 0, 0)

    def get_slot_tangent(self, slot_id):
        return Vector3(0, 1, 0)
