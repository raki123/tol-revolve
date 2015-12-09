from sdfbuilder.math import Vector3
from ..config import Config
from revolve.util import Time
from revolve.angle import Robot as RvRobot


class Robot(RvRobot):
    """
    Class to manage a single robot
    """

    def __init__(self, conf, gazebo_id, name, tree, robot, position, time, parents=None):
        """
        :param conf:
        :type conf: Config
        :param gazebo_id:
        :param name:
        :param tree:
        :param robot: Protobuf robot
        :param position:
        :type position: Vector3
        :param time:
        :type time: Time
        :param parents:
        :type parents: set
        :return:
        """
        super(Robot, self).__init__(gazebo_id=gazebo_id, name=name, tree=tree, robot=robot,
                                    position=position, time=time, speed_window=conf.speed_window,
                                    parents=parents)

        # Set of robots this bot has mated with
        self.mated_with = set()
        self.last_mate = None

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
