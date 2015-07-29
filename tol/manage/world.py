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
from revolve.gazebo import connect, RequestHandler, analysis_coroutine, get_analysis_robot
from revolve.spec.msgs import InsertSdfModelRequest
from revolve.angle import Tree, Crossover, Mutator
from revolve.util import Time
from sdfbuilder.math import Vector3
from sdfbuilder import SDF, Model


# Local
from ..config import Config, constants
from ..build import get_builder, get_simulation_robot
from ..spec import get_tree_generator
from ..util import multi_future
from .robot import Robot
from ..scenery import Wall

# Construct a message base from the time. This should make
# it unique enough for consecutive use when the script
# is restarted.
_a = time.time()
MSG_BASE = _a - 14*10e8 + (_a - int(_a)) * 10e5

# Seconds to wait between checking for answers in waiting loops
ANSWER_SLEEP = 0.05


class World(object):
    """
    A class that is used to manage the world, meaning it provides
    methods to insert / remove robots and request information
    about where they are.
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
        self.model_inserter = None
        self.builder = get_builder(conf)
        self.generator = get_tree_generator(conf)
        self.crossover = Crossover(self.generator.body_gen, self.generator.brain_gen)
        self.mutator = Mutator(self.generator.body_gen, self.generator.brain_gen)
        self.robots = {}
        self.robot_id = 0
        self.last_time = Time()

        # Set to true whenever a reproduction sequence is going on
        # to prevent another one from starting (which cannot happen now
        # but might in a more complicated yielding structure).
        self._reproducing = False

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
        self.analyzer = yield From(connect(self.conf.analyzer_address))
        self.world_control = yield From(self.manager.advertise(
            '/gazebo/default/world_control', 'gazebo.msgs.WorldControl'
        ))

        self.request_handler = yield From(RequestHandler.create(
            self.manager, msg_id_base=MSG_BASE))

        # Request handler for an insert robot flow. Note how this
        # uses the ~/model/info response with the name of the robot
        # to check whether it was inserted.
        self.model_inserter = yield From(RequestHandler.create(
            self.manager, request_class=InsertSdfModelRequest,
            response_class=model_pb2.Model,
            request_type='revolve.msgs.InsertSdfRequest',
            response_type='gazebo.msgs.ModelInfo',
            advertise='/gazebo/default/insert_robot_sdf/request',
            subscribe='/gazebo/default/model/info',
            id_attr='name'
        ))

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

        print("Failed to produce a valid robot in %d attempts." % max_attempts,
              file=sys.stderr)
        raise Return(None)

    @trollius.coroutine
    def analyze_tree(self, tree):
        """

        :param tree:
        :return:
        """
        robot = tree.to_robot(self.get_robot_id())
        sdf = get_analysis_robot(robot, builder=self.builder)

        ret = yield From(analysis_coroutine(sdf, self.analyzer))
        if ret is None:
            print("Error in robot analysis, continuing.", file=sys.stderr)
            raise Return(None)

        coll, bbox = ret
        raise Return((coll, bbox, robot))

    @trollius.coroutine
    def generate_starting_population(self, positions):
        """
        Generates and inserts a starting population of robots at the
        given positions.
        :param positions: Iterable of (x, y, z) positions, where z
                          is an offset from the z bounding box.
        :return:
        """
        to_insert = []
        for position in positions:
            gen = yield From(self.generate_valid_robot())
            if not gen:
                raise Return(None)

            tree, robot, bbox = gen
            insert_pos = Vector3(position) + Vector3(0, 0, bbox[2])
            to_insert.append((tree, insert_pos))

        futures = []
        for tree, position in to_insert:
            future = yield From(self.insert_robot(tree, position))
            futures.append(future)

        yield From(multi_future(futures))

    def get_robot_id(self):
        """
        Robot ID sequencer
        :return:
        """
        self.robot_id += 1
        return self.robot_id

    @trollius.coroutine
    def insert_robot(self, tree, position):
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
        :param position:
        :type position: Vector3
        :return:
        """
        robot_id = self.get_robot_id()
        robot_name = "gen__"+str(robot_id)
        robot = tree.to_robot(robot_id)
        sdf = get_simulation_robot(robot, robot_name, self.builder, self.conf)
        sdf.elements[0].set_position(position)

        future = yield From(self.insert_sdf(sdf))
        future.add_done_callback(lambda fut: self._robot_inserted(robot_name, tree, position,
                                                                  fut.result()))
        raise Return(future)

    def pause(self, pause=True):
        """
        Pause / unpause the world
        :param pause:
        :return: Future for the published message
        """
        msg = world_control_pb2.WorldControl()
        msg.pause = pause
        return self.world_control.publish(msg)

    def reset(self):
        """
        Reset the world
        :return:
        """
        msg = world_control_pb2.WorldControl()
        msg.reset.all = True
        return self.world_control.publish(msg)

    def _robot_inserted(self, robot_name, tree, position, msg):
        """
        Registers a newly inserted robot and marks the insertion
        message response as handled.

        :param tree:
        :param robot_name:
        :param position:
        :param msg:
        :return:
        """
        self.register_robot(msg.id, robot_name, tree, position)
        self.model_inserter.handled(robot_name)

    def register_robot(self, gazebo_id, robot_name, tree, position):
        """
        Registers a robot with its Gazebo ID in the local array.
        :param gazebo_id:
        :param tree:
        :param robot_name:
        :param position:
        :return:
        """
        print("Registering robot %s." % robot_name)
        self.robots[gazebo_id] = Robot(self.conf, gazebo_id, robot_name, tree, position)

    def _update_poses(self, msg):
        """
        Handles the pose info message by updating robot positions.
        :param msg:
        :return:
        """
        poses = poses_stamped_pb2.PosesStamped()
        poses.ParseFromString(msg)
        time = Time(msg=poses.time)

        for pose in poses.pose:
            robot = self.robots.get(pose.id, None)
            if not robot:
                continue

            position = Vector3(pose.position.x, pose.position.y, pose.position.z)
            robot.update_position(time, position)

    @trollius.coroutine
    def insert_sdf(self, sdf, callback=None):
        """
        Insert a model wrapped in an SDF tag into the world. Make
        sure it has a unique name, as this is used for the callback.

        This coroutine yields until the request has been successfully sent.
        It returns a future that resolves when a response has been received. The
        optional given callback is added to this future.

        :param sdf:
        :param callback: Response callback.
        :type sdf: SDF
        :return:
        """
        model = sdf.get_elements_of_type(Model)[0]
        """ :type: Model """

        msg = InsertSdfModelRequest()
        msg.name = model.name
        msg.sdf_contents = str(sdf)

        future = Future()
        if callback:
            future.add_done_callback(callback)

        yield From(self.model_inserter.do_request(msg, callback=future.set_result))
        raise Return(future)

    @trollius.coroutine
    def build_walls(self, points):
        """
        Builds a wall defined by the given points, used to shield the
        arena.
        :param points:
        :return:
        """
        futures = []
        l = len(points)
        for i in range(l):
            start = points[i]
            end = points[(i + 1) % l]
            wall = Wall("wall_%d" % i, start, end, constants.WALL_THICKNESS, constants.WALL_HEIGHT)
            future = yield From(self.insert_sdf(SDF(elements=[wall])))
            futures.append(future)

        raise Return(multi_future(futures))

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

        print("Found %d possible pairs for reproduction..." % len(mates))
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
        print("Attempting mating between `%s` and `%s`..." % (ra.name, rb.name))

        # Attempt to create a child through crossover
        success, child = self.crossover.crossover(ra.tree, rb.tree)
        if not success:
            print("Crossover failed.")
            raise Return(False)

        # Apply mutation
        print("Crossover succeeded, applying mutation...")
        self.mutator.mutate(child, in_place=True)

        # Check if the robot is valid
        ret = yield From(self.analyze_tree(child))
        if ret is None or ret[0]:
            print("Miscarriage.")
            raise Return(False)

        # Alright, we have a valid bot, let's add it
        # to the world.
        coll, bbox, robot = ret

        # We position the bot such that we get an equilateral
        # triangle `ra`, `rb`, `child`.
        # Get the vector from b to a
        diff = ra.last_position - rb.last_position
        diff.z = 0
        dist = diff.norm()

        # Calculate the vector normal to it
        if diff.y > 10e-5:
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

        print("New robot is viable, inserting.")

        # TODO Check if the position is within arena bounds
        future = yield From(self.insert_robot(child, pos))

        ra.did_mate_with(rb)
        rb.did_mate_with(ra)

        raise Return(future)
