from __future__ import absolute_import

# Revolve imports
from revolve.build.sdf import BodyPart, PID, PositionMotor
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.joint import Joint, Limit

# Local imports
from .util import ColorMixin
from .. import constants


class ActiveHinge(BodyPart, ColorMixin):
    """
    A passive hinge
    """
    MASS_SLOT = in_grams(7)
    MASS_SERVO = in_grams(9)
    MASS_FRAME = in_grams(1.2)
    SLOT_WIDTH = in_mm(34)
    SLOT_THICKNESS = in_mm(1.5)

    FRAME_LENGTH = in_mm(18)
    FRAME_HEIGHT = in_mm(10)
    FRAME_ROTATION_OFFSET = in_mm(14)
    """ Left to right """

    SERVO_LENGTH = in_mm(24.5)
    SERVO_HEIGHT = in_mm(10)
    SERVO_ROTATION_OFFSET = in_mm(20.5)
    """ Right to left """

    SEPARATION = in_mm(0.1)

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        # Create links
        self.hinge_root = self.create_link("slot_a")
        self.hinge_tail = self.create_link("slot_b")
        frame = self.create_link("frame")
        servo = self.create_link("servo")

        # Shorthand for variables
        separation = self.SEPARATION
        slot_mass = self.MASS_SLOT
        frame_mass = self.MASS_FRAME
        servo_mass = self.MASS_SERVO
        thickness = self.SLOT_THICKNESS
        width = self.SLOT_WIDTH
        frame_length = self.FRAME_LENGTH
        frame_height = self.FRAME_HEIGHT
        frame_rot = self.FRAME_ROTATION_OFFSET
        servo_length = self.SERVO_LENGTH
        servo_height = self.SERVO_HEIGHT
        servo_rot = self.SERVO_ROTATION_OFFSET

        # Initialize root properties
        self.hinge_root.make_box(slot_mass, thickness, width, width)

        # Make frame
        x_frame = thickness / 2.0 + separation + frame_length / 2.0
        frame.set_position(Vector3(x_frame, 0, 0))
        frame.make_box(frame_mass, frame_length, width, frame_height)

        # Make servo
        x_servo = x_frame + (frame_rot - 0.5 * frame_length) + \
                  (-0.5 * servo_length + servo_rot)
        servo.set_position(Vector3(x_servo, 0, 0))
        servo.make_box(servo_mass, servo_length, width, servo_height)

        # TODO Color servo

        # Make the tail
        x_tail = x_servo + servo_length / 2.0 + separation + thickness / 2.0
        self.hinge_tail.set_position(Vector3(x_tail, 0, 0))
        self.hinge_tail.make_box(slot_mass, thickness, width, width)

        # Create joints to hold the pieces in position
        # root <-> frame
        self.fix_links(self.hinge_root, frame, Vector3(-frame_length / 2.0, 0, 0),
                       Vector3(1, 0, 0))

        # Connection part a <(hinge)> connection part b
        # Hinge joint axis should point straight up, and anchor
        # the points in the center. Note that the position of a joint
        # is expressed in the child link frame, so we need to take the
        # position from the original code and subtract conn_b's position
        self.joint = Joint("revolute", servo, frame, axis=Vector3(0, 1, 0))
        self.joint.set_position(Vector3(-0.5 * frame_length + frame_rot, 0, 0))
        self.joint.axis.limit = Limit(
            lower=-constants.SERVO_LIMIT,
            upper=constants.SERVO_LIMIT,
            effort=constants.MAX_SERVO_TORQUE
        )
        self.add_joint(self.joint)

        # connection part b <-> tail
        self.fix_links(servo, self.hinge_tail, Vector3(-thickness / 2.0, 0, 0),
                       Vector3(1, 0, 0))

        # Now we add a motor that powers the joint. This particular servo
        # targets a position. Use a simple PID controller initially.
        pid = constants.SERVO_PID
        self.motors.append(PositionMotor(self.id, "rotate", self.joint, pid))

        # Apply color mixin
        self.apply_color()

    def get_slot_normal(self, slot_id):
        return self.get_slot_position(slot_id).normalized()

    def get_slot_tangent(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(0, 1, 0)

    def get_slot(self, slot_id):
        self.check_slot(slot_id)
        return self.hinge_root if slot_id == 0 else self.hinge_tail

    def get_slot_position(self, slot_id):
        self.check_slot(slot_id)

        offset = 0.5 * self.SLOT_THICKNESS
        if slot_id == 0:
            # The root slot is positioned half the slot
            # thickness to the left
            return Vector3(-offset, 0, 0)
        else:
            # A `BodyPart` is a posable group, so item positions are
            # in the parent frame. If we convert to the local frame we can
            # simply add the offset in the x-direction
            tail_pos = self.to_local_frame(self.hinge_tail.pose.position)
            return tail_pos + Vector3(offset, 0, 0)
