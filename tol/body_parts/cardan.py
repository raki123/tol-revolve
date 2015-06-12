# Revolve imports
from revolve.build.sdf import BodyPart
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.joint import Joint, Axis

# Local imports
from .util import ColorMixin

MASS_SLOT = in_grams(2)
MASS_FRAME = in_grams(3)

SLOT_WIDTH = in_mm(34)
SLOT_THICKNESS = in_mm(1.5)
CONNECTION_PART_LENGTH = in_mm(24)
CONNECTION_PART_HEIGHT = in_mm(10)

# Computed from the outermost corner of the connection part
CONNECTION_PART_OFFSET = in_mm(18.5)
SEPARATION = in_mm(0.1)


class Cardan(BodyPart, ColorMixin):
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
        conn_b = self.create_link("conn_b")
        self.tail = self.create_link("tail")

        # Initialize the root, already at 0, 0, 0
        self.root.make_box(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH)

        # Initialize the first connection part
        x_part_a = 0.5 * (SLOT_THICKNESS + CONNECTION_PART_LENGTH) + SEPARATION
        conn_a.make_box(MASS_FRAME, CONNECTION_PART_LENGTH, SLOT_WIDTH, CONNECTION_PART_HEIGHT)
        conn_a.set_position(Vector3(x_part_a, 0, 0))

        # Initialize the second connection part
        x_part_b = x_part_a - CONNECTION_PART_LENGTH + 2 * CONNECTION_PART_OFFSET
        conn_b.make_box(MASS_FRAME, CONNECTION_PART_LENGTH, CONNECTION_PART_HEIGHT, SLOT_WIDTH)
        conn_b.set_position(Vector3(x_part_b, 0, 0))

        # Initialize the tail
        x_tail = x_part_b + 0.5 * (CONNECTION_PART_LENGTH + SLOT_THICKNESS) + SEPARATION
        self.tail.make_box(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH)
        self.tail.set_position(Vector3(x_tail, 0, 0))

        # Fix the parts using joints
        # root <-> connection a
        self.fix_links(self.root, conn_a, Vector3(-0.5 * CONNECTION_PART_LENGTH, 0, 0),
                       Vector3(-1, 0, 0))

        # connection a <-> connection b
        cardan = Joint("universal", conn_a, conn_b, axis=Vector3(0, 0, 1))
        axis2 = Axis(axis=Vector3(0, 1, 0), tag_name="axis2")
        cardan.add_element(axis2)
        cardan.set_position(Vector3(0.5 * CONNECTION_PART_LENGTH - CONNECTION_PART_OFFSET))
        self.add_joint(cardan)

        # connection b <-> tail
        self.fix_links(conn_b, self.tail, Vector3(-0.5 * SLOT_THICKNESS, 0, 0),
                       Vector3(-1, 0, 0))

        # Apply color mixin
        self.apply_color()

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
