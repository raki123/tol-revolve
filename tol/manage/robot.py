from sdfbuilder.math import Vector3
from ..config import Config
from revolve.util import Time
import numpy as np


class Robot(object):
    """
    Class to manage a single robot
    """

    def __init__(self, conf, gazebo_id, name, tree, position, time):
        """
        :param conf:
        :type conf: Config
        :param gazebo_id:
        :param name:
        :param position:
        :type position: Vector3
        :param time:
        :type time: Time
        :return:
        """
        self.conf = conf
        self.tree = tree
        self.name = name
        self.gazebo_id = gazebo_id
        self.starting_position = position
        self.starting_time = time

        self.last_position = position
        self.last_update = time
        self.last_mate = None

        self._distances = np.zeros(conf.speed_window)
        self._times = np.zeros(conf.speed_window)
        self._dist = 0
        self._time = 0
        self._idx = 0

        # Set of robots this bot has mated with
        self.mated_with = set()

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

    def will_mate_with(self, other):
        """
        Decides whether or not to mate with the other given robot
        based on its position and speed.
        :param other:
        :type other: Robot
        :return:
        """
        if other in self.mated_with:
            # Don't mate with the same bot twice
            return False

        if self.last_mate is not None and \
           float(self.last_update - self.last_mate) < self.conf.mating_cooldown:
            # Don't mate within the cooldown window
            return False

        if not self.last_position or not other.last_position:
            return False

        dist = other.last_position - self.last_position
        dist.z = 0
        if dist.norm() > self.conf.mating_distance:
            return False

        my_vel = self.velocity()
        other_vel = other.velocity()

        return my_vel == 0 or (other_vel / my_vel) > self.conf.proposal_threshold

    def did_mate_with(self, other):
        """
        Called when this robot mated with another robot successfully.
        :param other:
        :return:
        """
        self.last_mate = self.last_update
        self.mated_with.add(other)

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


