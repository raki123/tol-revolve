# Revolve imports
from revolve.build.sdf import BodyPart, PositionMotor
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.joint import Joint, Limit

# Local imports
from .util import ColorMixin
from ..config import constants

MASS_SERVO = in_grams(7)
MASS_SLOT = in_grams(2)
MASS_CROSS = in_grams(3)

SLOT_WIDTH = in_mm(34)
SLOT_THICKNESS = in_mm(1.5)
CONNECTION_PART_WIDTH = in_mm(12.5)
CONNECTION_PART_LENGTH = in_mm(24.5)
CONNECTION_PART_HEIGHT = in_mm(21)
CROSS_WIDTH = in_mm(10)
CROSS_HEIGHT = in_mm(34)
CROSS_THICKNESS = in_mm(3)

# Outer to inner (was: left to right)
CONNECTION_PART_ROTATION_OFFSET = in_mm(20.5)

# Offset of the cross center from the rotation axis
CROSS_CENTER_OFFSET = in_mm(17)

# Offset of the cross center w.r.t. the center of the connection part
CONNECTION_PART_OFFSET = in_mm(19.875)

SEPARATION = in_mm(0.1)


class ActiveCardan(BodyPart, ColorMixin):
    """
    A passive cardan
    """

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        self.root = self.create_link("root")
        conn_a = self.create_link("conn_a")
        cross_a = self.create_link("cross_a")

        cross_a_edge_1 = self.create_link("cross_a_edge_1")
        cross_a_edge_2 = self.create_link("cross_a_edge_2")
        cross_b_edge_1 = self.create_link("cross_b_edge_1")
        cross_b_edge_2 = self.create_link("cross_b_edge_2")

        cross_b = self.create_link("cross_b")
        conn_b = self.create_link("conn_b")
        self.tail = self.create_link("tail")

        # Initialize the root, already at 0, 0, 0
        self.root.make_box(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH)

        # Initialize the first connection part
        x_part_a = 0.5 * (SLOT_THICKNESS + CONNECTION_PART_LENGTH) + SEPARATION
        conn_a.make_box(MASS_SERVO, CONNECTION_PART_LENGTH, CONNECTION_PART_WIDTH, CONNECTION_PART_HEIGHT)
        conn_a.set_position(Vector3(x_part_a, 0, 0))

        # Initialize the cross
        x_cross = x_part_a + CONNECTION_PART_OFFSET
        cross_pos = Vector3(x_cross, 0, 0)
        cross_a.make_box(MASS_CROSS / 3.0, CROSS_THICKNESS, CROSS_WIDTH, CROSS_HEIGHT)
        cross_b.make_box(MASS_CROSS / 3.0, CROSS_THICKNESS, CROSS_HEIGHT, CROSS_WIDTH)
        cross_a.set_position(cross_pos)
        cross_b.set_position(cross_pos)

        # Cross connection edges
        cross_edge_mass = MASS_CROSS / 12.0
        x_cross_a_edge = x_cross - 0.5 * (CROSS_WIDTH + CROSS_THICKNESS)
        z_cross_a_edge = 0.5 * (CROSS_HEIGHT + CROSS_THICKNESS)
        cross_a_edge_1.make_box(cross_edge_mass, CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS)
        cross_a_edge_1.set_position(Vector3(x_cross_a_edge, 0, z_cross_a_edge))
        cross_a_edge_2.make_box(cross_edge_mass, CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS)
        cross_a_edge_2.set_position(Vector3(x_cross_a_edge, 0, -z_cross_a_edge))

        x_cross_b_edge = x_cross + 0.5 * (CROSS_WIDTH + CROSS_THICKNESS)
        y_cross_b_edge = 0.5 * (CROSS_HEIGHT - CROSS_THICKNESS)
        cross_b_edge_1.make_box(cross_edge_mass, CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH)
        cross_b_edge_1.set_position(Vector3(x_cross_b_edge, y_cross_b_edge, 0))
        cross_b_edge_2.make_box(cross_edge_mass, CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH)
        cross_b_edge_2.set_position(Vector3(x_cross_b_edge, -y_cross_b_edge, 0))

        # Initialize the second connection part
        x_part_b = x_cross + CONNECTION_PART_OFFSET
        conn_b.make_box(MASS_SERVO, CONNECTION_PART_LENGTH, CONNECTION_PART_HEIGHT, CONNECTION_PART_WIDTH)
        conn_b.set_position(Vector3(x_part_b, 0, 0))

        # Initialize the tail
        x_tail = x_part_b + 0.5 * (CONNECTION_PART_LENGTH + SLOT_THICKNESS) + SEPARATION
        self.tail.make_box(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH)
        self.tail.set_position(Vector3(x_tail, 0, 0))

        # Fix the parts using joints
        # root <-> connection a
        self.fix_links(self.root, conn_a, Vector3(-0.5 * CONNECTION_PART_LENGTH, 0, 0),
                       Vector3(-1, 0, 0))

        # connection a <hinge> cross part a
        # easier to position if conn_a is the child
        joint_a = Joint("revolute", cross_a, conn_a, Vector3(0, 0, 1))
        joint_a.set_position(Vector3(-0.5 * CONNECTION_PART_LENGTH + CONNECTION_PART_ROTATION_OFFSET))
        joint_a.axis.limit = Limit(
            lower=-constants.SERVO_LIMIT,
            upper=constants.SERVO_LIMIT,
            effort=constants.MAX_SERVO_TORQUE
        )
        self.add_joint(joint_a)

        # cross part b <hinge> connection b
        joint_b = Joint("revolute", cross_b, conn_b, Vector3(0, 1, 0))
        joint_b.set_position(Vector3(0.5 * CONNECTION_PART_LENGTH - CONNECTION_PART_ROTATION_OFFSET))
        joint_a.axis.limit = Limit(
            lower=-constants.SERVO_LIMIT,
            upper=constants.SERVO_LIMIT,
            effort=constants.MAX_SERVO_TORQUE
        )
        self.add_joint(joint_b)

        # connection b <-> tail
        self.fix_links(conn_b, self.tail, Vector3(-0.5 * SLOT_THICKNESS, 0, 0),
                       Vector3(-1, 0, 0))

        # Fix the cross
        self.fix_links(cross_a, cross_b, Vector3(0, 0, 0), Vector3(1, 0, 0))
        self.fix_links(cross_a_edge_1, cross_a, Vector3(0, 0, 0.5 * CROSS_HEIGHT), Vector3(0, 0, 1))
        self.fix_links(cross_a_edge_2, cross_a, Vector3(0, 0, -0.5 * CROSS_HEIGHT), Vector3(0, 0, -1))
        self.fix_links(cross_b_edge_1, cross_b, Vector3(0, 0.5 * CROSS_HEIGHT, 0), Vector3(0, 1, 0))
        self.fix_links(cross_b_edge_2, cross_b, Vector3(0, -0.5 * CROSS_HEIGHT, 0), Vector3(0, -1, 0))

        # Apply color mixin
        self.apply_color()

        # Register motors
        pid = constants.SERVO_PID
        self.motors.append(PositionMotor(self.id, "motor_a", joint_a, pid=pid))
        self.motors.append(PositionMotor(self.id, "motor_b", joint_b, pid=pid))

    def get_slot(self, slot_id):
        self.check_slot(slot_id)
        return self.root if slot_id == 0 else self.tail

    def get_slot_position(self, slot_id):
        self.check_slot(slot_id)
        if slot_id == 0:
            return Vector3(-0.5 * SLOT_THICKNESS, 0, 0)
        else:
            return self.tail.to_sibling_frame(Vector3(0.5 * SLOT_THICKNESS, 0, 0), self)

    def get_slot_normal(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(-1, 0, 0) if slot_id == 0 else Vector3(1, 0, 0)

    def get_slot_tangent(self, slot_id):
        self.check_slot(slot_id)
        return Vector3(0, 1, 0)
