import ast
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

from pygazebo.pygazebo import DisconnectError
from trollius.py33_exceptions import ConnectionResetError

import trollius
from trollius import From

from tol.config import parser
from tol.manage import World

from sdfbuilder import Pose
from sdfbuilder.math import Vector3

from revolve.convert.yaml import yaml_to_robot
from revolve.angle import Tree
from revolve.util import wait_for

from tol.spec import get_extended_brain_spec	


@trollius.coroutine
def run():
    """
    The main coroutine, which is started below.
    """
    # Parse command line / file input arguments
    conf = parser.parse_args()

    # Adding brain configuration
    with open(conf.brain_conf_path, 'r') as f:
        s = f.read()
        brain_conf = ast.literal_eval(s)
    brain_conf["policy_load_path"] = conf.load_controller
    conf.brain_conf = brain_conf

    # This disables the analyzer; enable it if you want to generate valid robots
    # Can also do this using arguments of course, just pass an empty string
    # conf.analyzer_address = None

    with open("{}.yaml".format(conf.robot_name), 'r') as yamlfile:
        bot_yaml = yamlfile.read()

    # Create the world, this connects to the Gazebo world
    world = yield From(World.create(conf))

    # These are useful when working with YAML
    body_spec = world.builder.body_builder.spec
    brain_spec = get_extended_brain_spec(conf)

    # Create a robot from YAML
    robot = yaml_to_robot(body_spec, brain_spec, bot_yaml)

    # Create a revolve.angle `Tree` representation from the robot, which
    # is what is used in the world manager.
    robot_tree = Tree.from_body_brain(robot.body, robot.brain, body_spec)

    # Insert the robot into the world. `insert_robot` resolves when the insert
    # request is sent, the future it returns resolves when the robot insert
    # is actually confirmed and a robot manager object has been created
    pose = Pose(position=Vector3(0, 0, 0.05))
    future = yield From(world.insert_robot(robot_tree, pose, "{}-{}".format(conf.robot_name, conf.experiment_round)))
    robot_manager = yield From(future)

    # I usually start the world paused, un-pause it here. Note that
    # pause again returns a future for when the request is sent,
    # that future in turn resolves when a response has been received.
    # This is the general convention for all message actions in the
    # world manager. `wait_for` saves the hassle of grabbing the
    # intermediary future in this case.
    yield From(wait_for(world.pause(False)))

    # Start a run loop to do some stuff
    while True:
        # Print robot fitness every second
        print("Robot fitness: %f" % robot_manager.fitness())
        yield From(trollius.sleep(1.0))


def main():
    def handler(loop, context):
        exc = context['exception']
        if isinstance(exc, DisconnectError) or isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)
        raise context['exception']

    try:
        loop = trollius.get_event_loop()
        loop.set_exception_handler(handler)
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got CtrlC, shutting down.")


if __name__ == '__main__':
    main()
