from __future__ import print_function
import sys

# Revolve imports
from revolve.build.sdf import Box, Sensor
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.sensor import Sensor as SdfSensor
from sdfbuilder.util import number_format as nf

# Local imports
from .util import ColorMixin


class LightSensor(Box, ColorMixin):
    """
    Simple light sensor. This extends `Box` for convenience,
    make sure you set the arity to 1 in the body specification.
    """
    MASS = in_grams(2)
    SENSOR_BASE_WIDTH = in_mm(34)
    SENSOR_BASE_THICKNESS = in_mm(1.5)

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        self.x = self.SENSOR_BASE_THICKNESS
        self.y = self.z = self.SENSOR_BASE_WIDTH
        super(LightSensor, self)._initialize(**kwargs)

        # Add the SDF camera sensor
        camera = SdfSensor("%s_light_sensor" % self.id, "camera",
                           update_rate=self.conf.update_rate)
        cam_details = "<camera>" \
                      "<image>" \
                      "<width>1</width><height>1</height>" \
                      "</image>" \
                      "<clip><near>%s</near><far>%s</far></clip>" \
                      "</camera>" % (nf(in_mm(1)), nf(in_mm(50000)))
        camera.add_element(cam_details)
        camera.set_position(Vector3(0.5 * self.x, 0, 0))
        self.link.add_element(camera)

        if self.conf.visualize_sensors:
            camera.add_element("<visualize>1</visualize>")

        # Register the sensor
        sensor = Sensor(self.id, self.link, camera, "light")
        self.sensors.append(sensor)

        self.apply_color()

    def get_slot_position(self, slot):
        self.check_slot(slot)
        return Vector3(-0.5 * self.SENSOR_BASE_THICKNESS, 0, 0)

    def get_slot_tangent(self, slot):
        self.check_slot(slot)
        return Vector3(0, 1, 0)
