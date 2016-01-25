from __future__ import print_function

from revolve.angle import Tree
from revolve.spec.msgs import Robot
from revolve.util import wait_for

import os
import sys
import glob
import time
from revolve.util import Time
from sdfbuilder import Pose
from sdfbuilder.math import Vector3
import itertools

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

import trollius
from trollius import From
from tol.manage import World
from tol.config import parser
from tol.logging import logger
import logging


logger.setLevel(logging.DEBUG)


@trollius.coroutine
def sleep_sim_time(robot, seconds):
    while robot.age() < seconds:
        yield From(trollius.sleep(0.01))


@trollius.coroutine
def run():
    conf = parser.parse_args()
    conf.enable_light_sensor = False
    conf.output_directory = 'debug-output'
    conf.restore_dir = 'restore-test'

    world = yield From(World.create(conf))

    if world.do_restore:
        print("Restored world manager state.")
        yield From(wait_for(world.pause(False)))
        for robot in world.robot_list():
            print("%s: %f" % (robot.name, robot.velocity()))

        yield From(trollius.sleep(10.0))
        print("==============================")

        for robot in world.robot_list():
            print("%s: %f" % (robot.name, robot.velocity()))

    else:
        yield From(wait_for(world.pause()))
        trees, bboxes = yield From(world.generate_population(9))
        positions = itertools.product(range(3), repeat=2)

        for tree, bbox, position in itertools.izip(trees, bboxes, positions):
            x, y = position
            pos = Vector3(x, y, -bbox.min.z)
            yield From(wait_for(world.insert_robot(tree, Pose(position=pos))))

        yield From(wait_for(world.pause(False)))
        yield From(trollius.sleep(10.0))
        yield From(world.create_snapshot())

if __name__ == '__main__':
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")

