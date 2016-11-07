# Add "tol" directory to Python path
import random
import itertools
import os
import sys
import math
import time
import csv

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

# Trollius / Pygazebo
import trollius
from trollius import From, Return
from pygazebo.pygazebo import DisconnectError

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3
from sdfbuilder import Pose
from revolve.build.util import in_mm
from revolve.convert import yaml_to_robot
from revolve.angle.representation import Tree
from revolve.util import wait_for

# ToL
from tol.config import parser
from tol.manage import World
from tol.logging import logger, output_console
from tol.build import get_simulation_robot

# Output logs to console
output_console()


@trollius.coroutine
def yield_wait(call):
    """

    :param call:
    :return:
    """
    future = yield From(call)
    yield From(future)
    raise Return(future)


def get_poses(n, z=in_mm(24.5), spacing=0.4, row_limit=5):
    x = 0
    y = 0
    poses = []

    for i in range(n):
        poses.append(Pose(position=Vector3(x, y, z)))

        x += spacing
        if (i % row_limit) == 0:
            y += spacing
            x = 0

    return poses


@trollius.coroutine
def run_server():
    conf = parser.parse_args()
    conf.analyzer_address = None

    world = yield From(World.create(conf))
    yield From(world.pause(True))

    with open("/home/elte/mt/tol/scripts/starfish.yaml", "rb") as f:
        robot_yaml = f.read()

    body_spec = world.builder.body_builder.spec
    brain_spec = world.builder.brain_builder.spec
    bot = yaml_to_robot(body_spec, brain_spec, robot_yaml)

    fname = conf.output_directory+"/revolve_benchmark.csv"
    exists = os.path.exists(fname)
    if exists:
        f = open(fname, 'ab', buffering=1)
    else:
        f = open(fname, 'wb', buffering=1)

    output = csv.writer(f, delimiter=',')

    if not exists:
        output.writerow(['run', 'population_size', 'step_size',
                         'sim_time', 'real_time', 'factor'])

    n_bots = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
    sim_time = 5.0
    runs = 20

    yield From(world.pause(False))

    for n in n_bots:
        poses = get_poses(n)
        trees = [Tree.from_body_brain(bot.body, bot.brain, body_spec) for _ in range(n)]

        for i in range(runs):
            yield From(wait_for(world.insert_population(trees, poses)))

            while world.last_time is None:
                yield From(trollius.sleep(0.1))

            sim_before = world.last_time
            before = time.time()

            while float(world.last_time - sim_before) < sim_time:
                yield From(trollius.sleep(0.1))

            sim_diff = float(world.last_time - sim_before)
            diff = time.time() - before

            output.writerow((i, n, conf.world_step_size, sim_diff,
                             diff, sim_diff / diff))

            yield From(wait_for(world.delete_all_robots()))
            yield From(trollius.sleep(0.3))


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")
    except DisconnectError:
        print("World disconnected, shutting down.")

if __name__ == '__main__':
    main()

