# System imports
from __future__ import absolute_import

# SDF builder imports
from sdfbuilder.sensor import Sensor as SdfSensor
from sdfbuilder.math import Vector3
from sdfbuilder.structure import Box

# Revolve imports
from revolve.build.sdf import BodyPart, Sensor
from revolve.build.util import in_grams, in_mm

# Local imports
from .util import ColorMixin

MASS = in_grams(55.4)
WIDTH = in_mm(46.5)


class CoreComponent(BodyPart, ColorMixin):
    """
    The core component of the robot, basically a box with an IMU sensor.
    """

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        self.link = self.create_component(Box(WIDTH, WIDTH, WIDTH, MASS), "box")

        # Now we will add the IMU sensor. First, we must
        # create a sensor in SDF. Be careful to give the
        # sensor a name which is unique for the entire
        # robot, adding the ID will help us do that.
        imu = SdfSensor("imu_sensor", "imu", update_rate=self.conf.update_rate)
        self.link.add_sensor(imu)

        self.apply_color()

    def get_slot(self, slot):
        """
        There's only one slot, return the link.
        """
        self.check_slot(slot)
        return self.link

    def get_slot_position(self, slot):
        """
        Return slot position
        """
        self.check_slot(slot)
        vmax = WIDTH / 2.0
        if slot == 0:
            # Front face
            return Vector3(0, -vmax, 0)
        elif slot == 1:
            # Back face
            return Vector3(0, vmax, 0)
        elif slot == 2:
            # Top face
            return Vector3(0, 0, vmax)
        elif slot == 3:
            # Bottom face
            return Vector3(0, 0, -vmax)
        elif slot == 4:
            # Right face
            return Vector3(vmax, 0, 0)

        # Left face
        return Vector3(-vmax, 0, 0)

    def get_slot_normal(self, slot):
        """
        Return slot normal.
        """
        return self.get_slot_position(slot).normalized()

    def get_slot_tangent(self, slot):
        """
        Return slot tangent
        """
        self.check_slot(slot)
        if slot == 0:
            # Front face tangent: top face
            return Vector3(0, 0, 1)
        elif slot == 1:
            # Back face tangent: top face
            return Vector3(0, 0, 1)
        elif slot == 2:
            # Top face tangent: right face
            return Vector3(1, 0, 0)
        elif slot == 3:
            # Bottom face tangent: right face
            return Vector3(1, 0, 0)
        elif slot == 4:
            # Right face tangent: back face
            return Vector3(0, 1, 0)

        # Left face tangent: back face
        return Vector3(0, 1, 0)
