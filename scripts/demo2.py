# Add "tol" directory to Python path
import random
import itertools
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From
from pygazebo.pygazebo import DisconnectError

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3
from revolve.build.util import in_mm

# ToL
from tol.config import Config, constants
from tol.manage import World


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
        proposal_threshold=0
    )

    world = yield From(World.create(conf))
    yield From(world.pause(True))

    wall_x = conf.arena_size[0] / 2.0
    wall_y = conf.arena_size[1] / 2.0
    wall_points = [Vector3(-wall_x, -wall_y, 0), Vector3(wall_x, -wall_y, 0),
                   Vector3(wall_x, wall_y, 0), Vector3(-wall_x, wall_y, 0)]

    print("Generating walls...")
    future = yield From(world.build_walls(wall_points))
    yield From(future)
    print("Done.")

    grid_size = (2, 1)
    spacing = 10 * conf.mating_distance
    grid_x, grid_y = grid_size
    x_offset = -(grid_x - 1) * spacing * 0.5
    y_offset = -(grid_y - 1) * spacing * 0.5
    positions = [(x_offset + spacing * i, y_offset + spacing * j, 0)
                 for i, j in itertools.product(range(grid_x), range(grid_y))]

    print("Generating starting population...")
    yield From(world.generate_starting_population(positions))
    print("Done.")
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
    except DisconnectError:
        print("World disconnected, shutting down.")

if __name__ == '__main__':
    main()

