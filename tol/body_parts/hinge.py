# Revolve imports
from revolve.build.sdf import BodyPart
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.joint import Joint

# Local imports
from .util import ColorMixin


class Hinge(BodyPart, ColorMixin):
    """
    A passive hinge
    """
    MASS_SLOT = in_grams(2)
    MASS_FRAME = in_grams(1)
    SLOT_WIDTH = in_mm(34)
    SLOT_THICKNESS = in_mm(1.5)
    CONNECTION_PART_LENGTH = in_mm(20.5)
    CONNECTION_PART_HEIGHT = in_mm(20)
    CONNECTION_PART_THICKNESS = in_mm(2)
    CONNECTION_ROTATION_OFFSET = in_mm(18.5)
    """ Computed from the left corner of the connection part. """

    SEPARATION = in_mm(0.1)

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        # Create links
        self.hinge_root = self.create_link("slot_a")
        self.hinge_tail = self.create_link("slot_b")
        conn_a = self.create_link("conn_a")
        conn_b = self.create_link("conn_b")

        # Shorthand for variables
        separation = self.SEPARATION
        slot_mass = self.MASS_SLOT
        frame_mass = self.MASS_FRAME
        thickness = self.SLOT_THICKNESS
        width = self.SLOT_WIDTH
        cp_length = self.CONNECTION_PART_LENGTH
        cp_thickness = self.CONNECTION_PART_THICKNESS
        cp_height = self.CONNECTION_PART_HEIGHT
        cp_rot = self.CONNECTION_ROTATION_OFFSET

        # Initialize root properties
        self.hinge_root.make_box(slot_mass, thickness, width, width)

        # Position connection part a
        x_part_a = thickness / 2.0 + separation + cp_length / 2.0
        conn_a.set_position(Vector3(x_part_a, 0, 0))
        conn_a.make_box(frame_mass, cp_length, cp_thickness, cp_height)

        # Position connection part b
        x_part_b = x_part_a + (cp_length / 2.0 - (cp_length - cp_rot)) * 2
        conn_b.set_position(Vector3(x_part_b, 0, 0))
        conn_b.make_box(frame_mass, cp_length, cp_thickness, cp_height)

        # Make the tail
        x_tail = x_part_b + cp_length / 2.0 + separation + thickness / 2.0
        self.hinge_tail.set_position(Vector3(x_tail, 0, 0))
        self.hinge_tail.make_box(slot_mass, thickness, width, width)

        # Create joints to hold the pieces in position
        # root <-> connection part a
        self.fix_links(self.hinge_root, conn_a, Vector3(-cp_length / 2.0, 0, 0),
                       Vector3(1, 0, 0))

        # Connection part a <(hinge)> connection part b
        # Hinge joint axis should point straight up, and anchor
        # the points in the center. Note that the position of a joint
        # is expressed in the child link frame, so we need to take the
        # position from the original code and subtract conn_b's position
        self.joint = Joint("revolute", conn_a, conn_b, axis=Vector3(0, 0, 1))
        self.joint.set_position(Vector3(cp_length / 2.0 - cp_rot, 0, 0))
        self.add_joint(self.joint)

        # connection part b <-> tail
        self.fix_links(conn_b, self.hinge_tail, Vector3(-thickness / 2.0, 0, 0),
                       Vector3(1, 0, 0))

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
            # A `BodyPart` is a PosableGroup, so child positions are
            # simular to sibling positions. We can thus take the position
            # in the tail, and use sibling conversion to get the position
            # in the body part.
            return self.hinge_tail.to_sibling_frame(
                Vector3(offset, 0, 0),
                self
            )
