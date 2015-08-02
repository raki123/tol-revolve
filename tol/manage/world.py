# External / system
from __future__ import print_function, absolute_import
import random
import sys
import math
import trollius
from trollius import From, Return, Future
import time
import itertools

# Pygazebo
from pygazebo.msg import model_pb2, world_control_pb2, poses_stamped_pb2, world_reset_pb2

# Revolve / sdfbuilder
from revolve.gazebo import connect, RequestHandler, BodyAnalyzer
from revolve.angle import Tree, Crossover, Mutator
from revolve.util import Time
from revolve.spec.msgs import ModelInserted
from sdfbuilder.math import Vector3
from sdfbuilder import SDF, Model, Pose


# Local
from ..config import Config, constants
from ..build import get_builder, get_simulation_robot
from ..spec import get_tree_generator
from ..util import multi_future
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


class World(object):
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
    # Object used to make constructor private
    _PRIVATE = object()

    def __init__(self, conf, _private):
        """

        :param conf:
        :type conf: Config
        :return:
        """
        if _private is not self._PRIVATE:
            raise ValueError("The world cannot be directly constructed,"
                             "rather the `create` coroutine should be used.")

        self.conf = conf
        self.manager = None
        self.analyzer = None
        self.request_handler = None
        self.builder = get_builder(conf)
        self.generator = get_tree_generator(conf)
        self.crossover = Crossover(self.generator.body_gen, self.generator.brain_gen)
        self.mutator = Mutator(self.generator.body_gen, self.generator.brain_gen)
        self.robots = {}
        self.robot_id = 0

        self.start_time = None
        self.last_time = None

        # Set to true whenever a reproduction sequence is going on
        # to prevent another one from starting (which cannot happen now
        # but might in a more complicated yielding structure).
        self._reproducing = False

        # List of functions called when the local state updates
        self.update_triggers = []

    @trollius.coroutine
    def _init(self):
        """
        Initializes the world
        :return:
        """
        if self.manager is not None:
            return

        # Initialize the manager / analyzer connections as well as
        # the general request handler
        self.manager = yield From(connect(self.conf.world_address))
        self.analyzer = yield From(BodyAnalyzer.create(self.conf.analyzer_address))
        self.world_control = yield From(self.manager.advertise(
            '/gazebo/default/world_control', 'gazebo.msgs.WorldControl'
        ))

        self.request_handler = yield From(RequestHandler.create(
            self.manager, msg_id_base=MSG_BASE))

        # Subscribe to pose updates
        self.pose_subscriber = self.manager.subscribe(
            '/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self._update_poses)

        # Wait for connections
        yield From(self.world_control.wait_for_listener())
        yield From(self.pose_subscriber.wait_for_connection())

    @classmethod
    @trollius.coroutine
    def create(cls, conf):
        """
        Coroutine to create a world including connection
        operations. Use as:

        ```
        world = yield From(World.create(conf))
        ```

        :param conf:
        :type conf: Config
        :return:
        :rtype: World
        """
        self = cls(conf, cls._PRIVATE)
        yield From(self._init())
        raise Return(self)

    @trollius.coroutine
    def generate_valid_robot(self, max_attempts=100):
        """
        Uses tree generation in conjuction with the analyzer
        to generate a valid new robot.

        :param max_attempts:  Maximum number of tries before giving up.
        :type max_attempts: int
        :return:
        """
        for i in xrange(max_attempts):
            tree = self.generator.generate_tree()

            ret = yield From(self.analyze_tree(tree))
            if ret is None:
                # Error already shown
                continue

            coll, bbox, robot = ret
            if not coll:
                raise Return(tree, robot, bbox)

        logger.error("Failed to produce a valid robot in %d attempts." % max_attempts)
        raise Return(None)

    @trollius.coroutine
    def analyze_tree(self, tree):
        """
        Calls the body analyzer on a robot tree.
        :param tree:
        :return:
        """
        robot = tree.to_robot(self.get_robot_id())
        ret = yield From(self.analyzer.analyze_robot(robot, builder=self.builder))
        if ret is None:
            logger.error("Error in robot analysis, skipping.")
            raise Return(None)

        coll, bbox = ret
        raise Return(coll, bbox, robot)

    @trollius.coroutine
    def generate_starting_population(self, poses):
        """
        Generates and inserts a starting population of robots at the
        given positions. Robots will be inserted one by one, so
        it is probably a good idea to pause the world before doing this.
        :param poses: Iterable of (x, y, z) positions, where z
                          is an offset from the z bounding box.
        :type poses: list[Pose]
        :return: Future that resolves when all robots have been inserted.
        """
        logger.debug("Generating starting population...")
        futures = []
        for pose in poses:
            gen = yield From(self.generate_valid_robot())
            if not gen:
                raise Return(None)

            tree, robot, bbox = gen

            pose.position += Vector3(0, 0, bbox[2])
            future = yield From(self.insert_robot(tree, pose))
            futures.append(future)

        future = multi_future(futures)
        future.add_done_callback(lambda _: logger.debug("Done generating starting population."))
        raise Return(future)

    def get_robot_id(self):
        """
        Robot ID sequencer
        :return:
        """
        self.robot_id += 1
        return self.robot_id

    @trollius.coroutine
    def insert_robot(self, tree, pose):
        """
        Inserts a robot into the world. This consists of two steps:

        - Sending the insert request message
        - Receiving a ModelInfo response

        This method is a coroutine because of the first step, writing
        the message must be yielded since PyGazebo doesn't appear to
        support writing multiple messages simultaneously. For the response,
        i.e. the message that confirms the robot has been inserted, a
        future is returned.

        :param tree:
        :type tree: Tree
        :param pose:
        :type pose: Pose
        :return:
        """
        robot_id = self.get_robot_id()
        robot_name = "gen__"+str(robot_id)

        robot = tree.to_robot(robot_id)
        sdf = get_simulation_robot(robot, robot_name, self.builder, self.conf)
        sdf.elements[0].set_pose(pose)

        future = yield From(self.insert_model(sdf))
        future.add_done_callback(lambda fut: self._robot_inserted(robot_name, tree, fut.result()))
        raise Return(future)

    @trollius.coroutine
    def delete_robot(self, robot):
        """
        :param robot:
        :type robot: Robot
        :return:
        """
        # Immediately unregister the robot so no it won't be used
        # for anything else while it is being deleted.
        self.unregister_robot(robot)
        future = yield From(self.delete_model(robot.name))
        raise Return(future)

    @trollius.coroutine
    def delete_all_robots(self):
        """
        Deletes all robots from the world. Returns a future that resolves
        when all responses have been received.
        :return:
        """
        futures = []
        for bot in self.robots.values():
            future = yield From(self.delete_robot(bot))
            futures.append(future)

        raise Return(multi_future(futures))

    @trollius.coroutine
    def pause(self, pause=True):
        """
        Pause / unpause the world
        :param pause:
        :return: Future for the published message
        """
        logger.debug("Pausing the world.")
        msg = world_control_pb2.WorldControl()
        msg.pause = pause
        yield From(self.world_control.publish(msg))

    @trollius.coroutine
    def reset(self):
        """
        Reset the world
        :return:
        """
        logger.debug("Resetting the world state.")
        msg = world_control_pb2.WorldControl()
        msg.reset.all = True
        yield From(self.world_control.publish(msg))
        self.last_time = None
        self.start_time = None

    def _robot_inserted(self, robot_name, tree, msg):
        """
        Registers a newly inserted robot and marks the insertion
        message response as handled.

        :param tree:
        :param robot_name:
        :param msg:
        :type msg: pygazebo.msgs.response_pb2.Response
        :return:
        """
        inserted = ModelInserted()
        inserted.ParseFromString(msg.serialized_data)
        model = inserted.model
        gazebo_id = model.id
        time = Time(msg=inserted.time)
        p = model.pose.position
        position = Vector3(p.x, p.y, p.z)

        self.register_robot(gazebo_id, robot_name, tree, position, time)

    def register_robot(self, gazebo_id, robot_name, tree, position, time):
        """
        Registers a robot with its Gazebo ID in the local array.
        :param gazebo_id:
        :param tree:
        :param robot_name:
        :param position:
        :param time:
        :return:
        """
        logger.debug("Registering robot %s." % robot_name)
        self.robots[gazebo_id] = Robot(self.conf, gazebo_id, robot_name, tree, position, time)

    def unregister_robot(self, robot):
        """
        Unregisters the robot with the given ID, usually happens when
        it is deleted.
        :param robot:
        :type robot: Robot
        :return:
        """
        logger.debug("Unregistering robot %s." % robot.name)
        del self.robots[robot.gazebo_id]

    def _update_poses(self, msg):
        """
        Handles the pose info message by updating robot positions.
        :param msg:
        :return:
        """
        poses = poses_stamped_pb2.PosesStamped()
        poses.ParseFromString(msg)

        self.last_time = t = Time(msg=poses.time)
        if self.start_time is None:
            self.start_time = t

        for pose in poses.pose:
            robot = self.robots.get(pose.id, None)
            if not robot:
                continue

            position = Vector3(pose.position.x, pose.position.y, pose.position.z)
            robot.update_position(t, position)

        self.call_update_triggers()

    def add_update_trigger(self, callback):
        """
        Adds an update trigger, a function called every time the local
        state is updated.
        :param callback:
        :type callback: callable
        :return:
        """
        self.update_triggers.append(callback)

    def remove_update_trigger(self, callback):
        """
        Removes a previously installed update trigger.
        :param callback:
        :type callback: callable
        :return:
        """
        self.update_triggers.remove(callback)

    def call_update_triggers(self):
        """
        Calls all update triggers.
        :return:
        """
        for callback in self.update_triggers:
            callback(self)

    @trollius.coroutine
    def insert_model(self, sdf):
        """
        Insert a model wrapped in an SDF tag into the world. Make
        sure it has a unique name, as it will be literally inserted into the world.

        This coroutine yields until the request has been successfully sent.
        It returns a future that resolves when a response has been received. The
        optional given callback is added to this future.

        :param sdf:
        :type sdf: SDF
        :return:
        """
        future = yield From(self.request_handler.do_gazebo_request("insert_sdf", data=str(sdf)))
        raise Return(future)

    @trollius.coroutine
    def delete_model(self, name):
        """
        Deletes the model with the given name from the world.
        :param name:
        :return:
        """
        future = yield From(self.request_handler.do_gazebo_request("entity_delete", data=name))
        raise Return(future)

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

            future = yield From(self.mate(ra, rb))

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
    def mate(self, ra, rb):
        """
        Coroutine that performs a mating attempt between robots `ra` and `rb`.
        :param ra:
        :type ra: Robot
        :param rb:
        :type rb: Robot
        :return: Returns `False` if mating failed, or a future with the robot that
                 is being inserted otherwise.
        """
        logger.debug("Attempting mating between `%s` and `%s`..." % (ra.name, rb.name))

        # Attempt to create a child through crossover
        success, child = self.crossover.crossover(ra.tree, rb.tree)
        if not success:
            logger.debug("Crossover failed.")
            raise Return(False)

        # Set the child's parents for future reference
        child.parents.update([ra, rb])

        # Apply mutation
        logger.debug("Crossover succeeded, applying mutation...")
        self.mutator.mutate(child, in_place=True)

        # Check if the robot is valid
        ret = yield From(self.analyze_tree(child))
        if ret is None or ret[0]:
            logger.debug("Miscarriage.")
            raise Return(False)

        # Alright, we have a valid bot, let's add it
        # to the world.
        logger.debug("New robot is viable, inserting.")
        _, bbox, robot = ret

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
        drop_length = sign * 0.5 * math.sqrt(3.0) * dist
        pos = rb.last_position + (0.5 * diff) + drop_length * normal
        pos.z = bbox[2]

        # TODO Check if the position is within arena bounds
        future = yield From(self.insert_robot(child, Pose(position=pos)))

        ra.did_mate_with(rb)
        rb.did_mate_with(ra)

        raise Return(future)
