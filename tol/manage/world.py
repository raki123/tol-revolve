# External / system
from __future__ import print_function, absolute_import
import sys
import trollius
from trollius import From, Return, Future
import time
from pygazebo.msg import model_pb2, world_control_pb2

# Revolve / sdfbuilder
from revolve.gazebo import connect, RequestHandler, analysis_coroutine, get_analysis_robot
from revolve.spec.msgs import InsertSdfRobotRequest
from revolve.angle import Tree
from sdfbuilder.math import Vector3

# Local
from ..config import Config
from ..build import get_builder, get_simulation_robot
from ..spec import get_tree_generator
from ..util import multi_future


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
        self.robot_creator = None
        self.builder = get_builder(conf)
        self.generator = get_tree_generator(conf)
        self.robots = {}
        self.robot_id = 0

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
        self.world_control = yield From(self.manager.advertise('/default/gazebo/world_control',
                                                               'gazebo.msgs.WorldControl'))
        self.request_handler = yield From(RequestHandler.create(
            self.manager, msg_id_base=MSG_BASE))

        # Request handler for an insert robot flow. Note how this
        # uses the ~/model/info response with the name of the robot
        # to check whether it was inserted.
        self.robot_creator = yield From(RequestHandler.create(
            self.manager, request_class=InsertSdfRobotRequest,
            response_class=model_pb2.Model,
            request_type='revolve.msgs.InsertSdfRequest',
            response_type='gazebo.msgs.ModelInfo',
            advertise='/gazebo/default/insert_robot_sdf/request',
            subscribe='/gazebo/default/model/info',
            id_attr='name'
        ))

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
            self.robot_id += 1
            robot = tree.to_robot(self.robot_id)
            sdf = get_analysis_robot(robot, builder=self.builder)

            ret = yield From(analysis_coroutine(sdf, self.analyzer))
            if ret is None:
                print("Error in robot analysis, continuing.", file=sys.stderr)
                continue

            coll, bbox = ret
            if not coll:
                raise Return(tree, robot, bbox)

        print("Failed to produce a valid robot in %d attempts." % max_attempts,
              file=sys.stderr)
        raise Return(None)

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

        for tree, position in to_insert:
            yield From(self.insert_robot(tree, position))
        # futures = [self.insert_robot(tree, position) for tree, position in to_insert]
        # yield From(multi_future(futures))

    def get_robot_id(self):
        """
        Robot ID sequencer
        :return:
        """
        self.robot_id += 1
        return self.robot_id

    def insert_robot(self, tree, position):
        """
        :param tree:
        :type tree: Tree
        :param position:
        :type position: Vector3
        :return:
        """
        robot_id = self.get_robot_id()
        robot_name = "gen__"+str(robot_id)
        robot = tree.to_robot(robot_id)
        msg = InsertSdfRobotRequest()
        msg.name = robot_name
        sdf = get_simulation_robot(robot, robot_name, self.builder, self.conf)
        sdf.elements[0].set_position(position)
        msg.sdf_contents = str(sdf)

        future = Future()
        self.robot_creator.do_request(msg, callback=future.set_result)

        # Mark the message as handled upon receiving the callback future
        future.add_done_callback(lambda _: self.robot_creator.handled(robot_name))
        return future

    def pause(self, paused=True):
        """
        Pauses / unpauses the world
        :param paused:
        :return:
        """
        msg = world_control_pb2.WorldControl()
        msg.pause = paused
        return self.world_control.publish(msg)

    @trollius.coroutine
    def list_entities(self):
        """
        Performs an `entity_list` request to the world and yields
        the returned response.
        :return:
        """
        rq = self.request_handler
        msg_id = rq.get_msg_id()
        future = Future()
        yield From(rq.do_gazebo_request(msg_id, data="entity_list",
                                        callback=future.set_result))
        rq.handled(msg_id)
        raise Return(future.result())
