from sdfbuilder.math import Vector3
from ..config import Config


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
        self.position = position
        self.name = name
        self.gazebo_id = gazebo_id

    def update_position(self, time, position):
        """

        :param time:
        :param position:
        :type position: Vector3
        :return:
        """
        self.position = position
