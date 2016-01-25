# External / system
from __future__ import print_function, absolute_import
import random
import sys
import math
import trollius
from trollius import From, Return, Future
import time
import itertools
import csv
import os
from datetime import datetime

# Pygazebo
from pygazebo.msg import world_control_pb2, poses_stamped_pb2, world_stats_pb2

# Revolve / sdfbuilder
from revolve.angle import Tree, Crossover, Mutator, WorldManager
from sdfbuilder.math import Vector3
from sdfbuilder import SDF, Model, Pose, Link


# Local
from ..config import constants, parser, str_to_address
from ..build import get_builder, get_simulation_robot
from ..spec import get_tree_generator
from revolve.util import multi_future
from .robot import Robot
from ..scenery import Wall
from ..logging import logger

# Construct a message base from the time. This should make
# it unique enough for consecutive use when the script
# is restarted.
_a = time.time()
MSG_BASE = int(_a - 14e8 + (_a - int(_a)) * 1e5)

# Seconds to wait between checking for answers in waiting loops
ANSWER_SLEEP = 0.05


class World(WorldManager):
    """
    A class that is used to manage the world, meaning it provides
    methods to insert / remove robots and request information
    about where they are.

    The world class contains a number of coroutines, usually from
    a request / response perspective. These methods thus work with
    two futures - one for the request to complete, one for the
    response to arrive. The convention for these methods is to
    always yield the first future, because it has proven problematic
    to send multiple messages over the same channel, so a request
    is always sent until completion. The methods then return the
    future that resolves when the response is delivered.
    """

    def __init__(self, conf, _private):
        """

        :param conf:
        :return:
        """
        super(World, self).__init__(_private=_private,
                                    world_address=str_to_address(conf.world_address),
                                    analyzer_address=str_to_address(conf.analyzer_address),
                                    output_directory=conf.output_directory,
                                    builder=get_builder(conf),
                                    pose_update_frequency=conf.pose_update_frequency,
                                    generator=get_tree_generator(conf),
                                    restore=conf.restore_dir)

        self.conf = conf
        self.crossover = Crossover(self.generator.body_gen, self.generator.brain_gen)
        self.mutator = Mutator(self.generator.body_gen, self.generator.brain_gen,
                               p_duplicate_subtree=conf.p_duplicate_subtree,
                               p_swap_subtree=conf.p_swap_subtree,
                               p_delete_subtree=conf.p_delete_subtree,
                               p_remove_brain_connection=conf.p_remove_brain_connection,
                               p_delete_hidden_neuron=conf.p_delete_hidden_neuron)

        # Set to true whenever a reproduction sequence is going on
        # to prevent another one from starting (which cannot happen now
        # but might in a more complicated yielding structure).
        self._reproducing = False

        # Write settings to config file
        if self.output_directory:
            parser.write_to_file(conf, os.path.join(self.output_directory, "settings.conf"))

    @classmethod
    @trollius.coroutine
    def create(cls, conf):
        """
        Coroutine to instantiate a Revolve.Angle WorldManager
        :param conf:
        :return:
        """
        self = cls(_private=cls._PRIVATE, conf=conf)
        yield From(self._init())
        raise Return(self)

    def create_robot_manager(self, robot_name, tree, robot, position, time, parents):
        """
        Overriding with robot manager with more capabilities.
        :param robot_name:
        :param tree:
        :param robot:
        :param position:
        :param time:
        :param parents:
        :return:
        """
        return Robot(self.conf, robot_name, tree, robot, position, time, parents)

    @trollius.coroutine
    def add_highlight(self, position, color):
        """
        Adds a circular highlight at the given position.
        :param position:
        :param color:
        :return:
        """
        hl = Highlight("highlight_"+str(self.get_robot_id()), color)
        position = position.copy()
        position.z = 0
        hl.set_position(position)
        sdf = SDF(elements=[hl])
        fut = yield From(self.insert_model(sdf))
        raise Return(fut, hl)

    @trollius.coroutine
    def generate_population(self, n):
        """
        Generates population of `n` valid robots robots.

        :param n: Number of robots
        :return: Future with a list of valid robot trees and corresponding
                 bounding boxes.
        """
        logger.debug("Generating population...")
        trees = []
        bboxes = []

        for _ in xrange(n):
            gen = yield From(self.generate_valid_robot())
            if not gen:
                raise Return(None)

            tree, robot, bbox = gen
            trees.append(tree)
            bboxes.append(bbox)

        raise Return(trees, bboxes)

    @trollius.coroutine
    def insert_population(self, trees, poses):
        """
        :param trees:
        :type trees: list[Tree]
        :param poses: Iterable of (x, y, z) positions to insert.
        :type poses: list[Pose]
        :return:
        """
        futures = []
        for tree, pose in itertools.izip(trees, poses):
            future = yield From(self.insert_robot(tree, pose))
            futures.append(future)

        future = multi_future(futures)
        future.add_done_callback(lambda _: logger.debug("Done inserting population."))
        raise Return(future)

    def get_simulation_sdf(self, robot, robot_name):
        """
        :param robot:
        :param robot_name:
        :return:
        """
        return get_simulation_robot(robot, robot_name, self.builder, self.conf)

    @trollius.coroutine
    def build_walls(self, points):
        """
        Builds a wall defined by the given points, used to shield the
        arena.
        :param points:
        :return: Future that resolves when all walls have been inserted.
        """
        futures = []
        l = len(points)
        for i in range(l):
            start = points[i]
            end = points[(i + 1) % l]
            wall = Wall("wall_%d" % i, start, end, constants.WALL_THICKNESS, constants.WALL_HEIGHT)
            future = yield From(self.insert_model(SDF(elements=[wall])))
            futures.append(future)

        raise Return(multi_future(futures))

    @trollius.coroutine
    def build_arena(self):
        """
        Initializes the arena by building square arena wall blocks.
        :return: Future that resolves when arena building is done.
        """
        logger.debug("Building the arena...")
        wall_x = self.conf.arena_size[0] / 2.0
        wall_y = self.conf.arena_size[1] / 2.0
        wall_points = [Vector3(-wall_x, -wall_y, 0), Vector3(wall_x, -wall_y, 0),
                       Vector3(wall_x, wall_y, 0), Vector3(-wall_x, wall_y, 0)]

        future = yield From(self.build_walls(wall_points))
        future.add_done_callback(lambda _: logger.debug("Done building the arena."))
        raise Return(future)

    @trollius.coroutine
    def perform_reproduction(self):
        """
        Selects all possible mating pairs and attempts to perform reproduction
        with them.
        :return:
        """
        if self._reproducing:
            return

        mates = self.select_mates()
        if not mates:
            return

        self._reproducing = True
        mates = self.select_mates()

        logger.debug("Found %d possible pairs for reproduction..." % len(mates))
        mated = set()
        futures = []

        for ra, rb in mates:
            if ra in mated or rb in mated:
                # Don't reproduce with the same robot
                # more than once in a loop.
                continue

            mated.add(ra)
            mated.add(rb)

            future = yield From(self.mate_and_insert(ra, rb))

            if future:
                futures.append(future)

        # All reproductions are done, the only thing left is insert the robots
        self._reproducing = False

        if futures:
            yield From(multi_future(futures))

    def select_mates(self):
        """
        Finds all mate combinations in the current arena.
        :return:
        """
        robot_list = self.robots.values()
        return [(ra, rb) for ra, rb in itertools.combinations(robot_list, 2)
                if ra.will_mate_with(rb) and rb.will_mate_with(ra)]

    @trollius.coroutine
    def attempt_mate(self, ra, rb):
        """
        Attempts mating between two robots.
        :param ra:
        :param rb:
        :return:
        """
        logger.debug("Attempting mating between `%s` and `%s`..." % (ra.name, rb.name))

        # Attempt to create a child through crossover
        success, child = self.crossover.crossover(ra.tree, rb.tree)
        if not success:
            logger.debug("Crossover failed.")
            raise Return(False)

        # Apply mutation
        logger.debug("Crossover succeeded, applying mutation...")
        self.mutator.mutate(child, in_place=True)

        # Check if the robot is valid
        ret = yield From(self.analyze_tree(child))
        if ret is None or ret[0]:
            logger.debug("Intersecting body parts: Miscarriage.")
            raise Return(False)

        logger.debug("Viable child created.")
        raise Return(child, ret[1])

    @trollius.coroutine
    def mate_and_insert(self, ra, rb):
        """
        Coroutine that performs a mating attempt between robots `ra` and `rb`.
        :param ra:
        :type ra: Robot
        :param rb:
        :type rb: Robot
        :return: Returns `False` if mating failed, or a future with the robot that
                 is being inserted otherwise.
        """
        ret = yield From(self.attempt_mate(ra, rb))
        if ret is False:
            raise Return(False)

        # Alright, we have a valid bot, let's add it
        # to the world.
        logger.debug("New robot is viable, inserting.")
        child, bbox = ret

        pos = self.get_equilateral_position(ra, rb)

        # TODO Check if the position is within arena bounds
        future = yield From(self.insert_robot(child, Pose(position=pos), parents=[ra, rb]))

        ra.did_mate_with(rb)
        rb.did_mate_with(ra)

        raise Return(future)

    def get_equilateral_position(self, ra, rb, mult=1.0):
        """

        :param ra:
        :param rb:
        :param mult: Multiply actual distance vector by this value.
        :return:
        """
        # We position the bot such that we get an equilateral
        # triangle `ra`, `rb`, `child`.
        # Get the vector from b to a
        diff = ra.last_position - rb.last_position
        diff.z = 0
        dist = diff.norm()

        # Calculate the vector normal to it. Make sure we're
        # not dividing by zero.
        if abs(diff.y) > 10e-5:
            normal = Vector3(1, diff.x / -diff.y, 0).normalized()
        else:
            normal = Vector3(-diff.y / diff.x, 1, 0).normalized()

        # Traverse half the difference vector, plus the length of
        # the drop line in the equilateral triangle along the
        # normal vector.
        sign = 1 if random.random() < 0.5 else -1
        drop_length = mult * sign * 0.5 * math.sqrt(3.0) * dist
        pos = rb.last_position + (0.5 * diff) + drop_length * normal
        pos.z = 0
        return pos


class Highlight(Model):
    """
    Model to highlight newly inserted robots / selected parents
    """

    def __init__(self, name, color, **kwargs):
        super(Highlight, self).__init__(name, static=True, **kwargs)
        self.highlight = Link("hl_link")
        self.highlight.make_cylinder(10e10, 0.4, 0.001, collision=False)
        r, g, b, a = color
        self.highlight.make_color(r, g, b, a)
        self.add_element(self.highlight)
