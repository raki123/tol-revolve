# Revolve imports
from revolve.build.sdf import Box, Sensor
from revolve.build.util import in_grams, in_mm

# SDF builder imports
from sdfbuilder.math import Vector3
from sdfbuilder.sensor import Sensor as SdfSensor

# Local imports
from .util import ColorMixin


class TouchSensor(Box, ColorMixin):
    """
    Simple light sensor. This extends `Box` for convenience,
    make sure you set the arity to 1 in the body specification.
    """
    MASS = in_grams(3)
    SENSOR_BASE_WIDTH = in_mm(34)
    SENSOR_BASE_THICKNESS = in_mm(1.5)
    SENSOR_THICKNESS = in_mm(9)
    SENSOR_WIDTH = in_mm(18.5)
    SENSOR_HEIGHT = in_mm(16)
    """ Of each left / right sensors """

    def _initialize(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        self.x = self.SENSOR_BASE_THICKNESS
        self.y = self.z = self.SENSOR_BASE_WIDTH
        super(TouchSensor, self)._initialize(**kwargs)

        x_sensors = 0.5 * (self.SENSOR_BASE_THICKNESS + self.SENSOR_THICKNESS)
        y_left_sensor = -0.5 * self.SENSOR_WIDTH - in_mm(1)
        y_right_sensor = 0.5 * self.SENSOR_WIDTH + in_mm(1)

        self._sensor_helper("left_sensor", x_sensors, y_left_sensor)
        self._sensor_helper("right_sensor", x_sensors, y_right_sensor)

        self.apply_color()

    def _sensor_helper(self, label, x_sensors, y_sensor):
        """
        :return:
        """
        sensor_link = self.create_link(label)
        sensor_link.make_box(self.MASS, self.SENSOR_THICKNESS, self.SENSOR_WIDTH,
                             self.SENSOR_HEIGHT)

        sensor_link.set_position(Vector3(x_sensors, y_sensor, 0))

        # Anchor and axis are in child frame
        anchor = Vector3(-0.5 * self.SENSOR_THICKNESS, 0, 0)
        axis = Vector3(1, 0, 0)
        self.fix_links(self.link, sensor_link, anchor, axis)

        # Create and add the SDF sensor
        contact = SdfSensor(label+"_sensor", "contact",
                            update_rate=self.conf.update_rate)
        sensor_link.add_element(contact)

        # Register the sensor
        sensor = Sensor(self.id, sensor_link, contact, "touch")
        self.sensors.append(sensor)

    def get_slot_position(self, slot):
        self.check_slot(slot)
        return Vector3(-0.5 * self.SENSOR_BASE_THICKNESS, 0, 0)

    def get_slot_tangent(self, slot):
        self.check_slot(slot)
        return Vector3(0, 1, 0)