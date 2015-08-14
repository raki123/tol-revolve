# Add "tol" directory to Python path
from __future__ import absolute_import
import random
import itertools
import os
import sys
import logging

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From
from pygazebo.pygazebo import DisconnectError

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3
from sdfbuilder import Pose
from revolve.build.util import in_mm

# ToL
from tol.config import Config, constants
from tol.manage import World
from tol.logging import logger, output_console

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Set command line seed if supplied, otherwise choose a random number
if len(sys.argv) > 1:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 1000000)

random.seed(seed)
print("Seed: %d" % seed)


@trollius.coroutine
def run_server():
    conf = Config(
        update_rate=25,
        proposal_threshold=0,
        output_directory='/home/elte/tol-out-test'
    )

    world = yield From(World.create(conf))
    yield From(world.pause(True))

    wall_x = conf.arena_size[0] / 2.0
    wall_y = conf.arena_size[1] / 2.0
    wall_points = [Vector3(-wall_x, -wall_y, 0), Vector3(wall_x, -wall_y, 0),
                   Vector3(wall_x, wall_y, 0), Vector3(-wall_x, wall_y, 0)]

    future = yield From(world.build_walls(wall_points))
    yield From(future)

    grid_size = (3, 3)
    spacing = 3 * conf.mating_distance
    grid_x, grid_y = grid_size
    x_offset = -(grid_x - 1) * spacing * 0.5
    y_offset = -(grid_y - 1) * spacing * 0.5
    positions = [Pose(position=Vector3(x_offset + spacing * i, y_offset + spacing * j, 0))
                 for i, j in itertools.product(range(grid_x), range(grid_y))]

    yield From(world.generate_starting_population(positions))
    # yield From(world.pause(False))

    while True:
        yield From(trollius.sleep(0.1))
        yield From(world.perform_reproduction())


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")

if __name__ == '__main__':
    main()

