from sdfbuilder.math import Vector3
from ..config import Config
from revolve.util import Time
import numpy as np


class Robot(object):
    """
    Class to manage a single robot
    """
    def __init__(self, conf, gazebo_id, name, tree, position):
        """
        :param conf:
        :type conf: Config
        :param gazebo_id:
        :param name:
        :param position:
        :type position: Vector3
        :return:
        """
        self.conf = conf
        self.tree = tree
        self.name = name
        self.gazebo_id = gazebo_id
        self.starting_position = position

        self.starting_time = None
        self.last_update = None
        self.last_position = None

        self._distances = np.zeros(conf.speed_window)
        self._times = np.zeros(conf.speed_window)
        self._dist = 0
        self._time = 0
        self._idx = 0

    def update_position(self, time, position):
        """

        :param time: The simulation time at the time of this
                     position update.
        :type time: Time
        :param position:
        :type position: Vector3
        :return:
        """
        if self.starting_time is None:
            self.starting_time = time
            self.last_update = time
            self.last_position = position

        # Calculate the distance the robot has covered as the Euclidean distance over
        # the x and y coordinates (we don't care for flying), as well as the time
        # it took to cover this distance.
        last = self.last_position
        ds = np.sqrt((position.x - last.x)**2 + (position.y - last.y)**2)
        dt = float(time - self.last_update)

        # Velocity is of course sum(distance) / sum(time)
        # Storing all separate distance and time values allows us to
        # efficiently calculate the new speed over the window without
        # having to sum the entire arrays each time.
        idx = self._idx
        self._dist += ds - self._distances[idx]
        self._time += dt - self._times[idx]

        self._distances[idx] = ds
        self._times[idx] = dt

        # Update the slot for the next value in the sliding window
        self._idx = (self._idx + 1) % len(self._distances)

        self.last_position = position
        self.last_update = time

    def velocity(self):
        """
        Returns the velocity over the maintained window
        :return:
        """
        return self._dist / self._time if self._time > 0 else 0

    def age(self):
        """
        Returns this robot's age as a Time object.
        Depends on the last and first update times.
        :return:
        :rtype: Time
        """
        return Time() if self.last_update is None else self.last_update - self.starting_time


