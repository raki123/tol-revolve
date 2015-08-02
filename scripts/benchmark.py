# Add "tol" directory to Python path
import random
import itertools
import os
import sys
import math
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return
from pygazebo.pygazebo import DisconnectError

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3, Quaternion
from sdfbuilder import Pose
from revolve.build.util import in_mm

# ToL
from tol.config import Config, constants
from tol.manage import World
from tol.logging import logger, output_console

# Output logs to console
output_console()

# Set command line seed if supplied, otherwise choose a random number
if len(sys.argv) > 1:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 1000000)

random.seed(seed)
logger.debug("Seed: %d" % seed)


def get_circle_poses(n, radius):
    """

    :param n:
    :param radius:
    :return:
    """
    shift = 2.0 * math.pi / n
    poses = []
    for i in xrange(n):
        angle = i * shift
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        starting_rotation = Quaternion.from_angle_axis(math.pi + angle, Vector3(0, 0, 1))
        poses.append(Pose(position=Vector3(x, y, 0), rotation=starting_rotation))

    return poses


@trollius.coroutine
def yield_wait(call):
    """

    :param call:
    :return:
    """
    future = yield From(call)
    yield From(future)
    raise Return(future)


@trollius.coroutine
def run_server():
    conf = Config(
        update_rate=25,
        proposal_threshold=0
    )

    world = yield From(World.create(conf))
    yield From(world.pause(True))
    yield From(yield_wait(world.build_arena()))

    n_bots = [1, 5, 20, 50]
    radius = 0.4 * conf.arena_size[0]
    n_repeats = 1
    sim_time = 5.0

    _state = [None, -1]

    # World update trigger
    def trigger(_):
        elapsed = float(world.last_time)
        if elapsed >= sim_time:
            _state[0] = time.time() - _state[0]
            _state[1] = elapsed

            # Remove trigger to prevent other incoming messages
            # from overwriting these values.
            world.remove_update_trigger(trigger)
        elif elapsed < 0:
            logger.error("Elapsed time < 0!")

    print("nbots\titer\tsimtime\twctime")
    for n in n_bots:
        poses = get_circle_poses(n, radius)

        for i in range(n_repeats):
            # Generate a starting population from the given poses
            yield From(yield_wait(world.generate_starting_population(poses)))
            yield From(trollius.sleep(0.5))
            _state[0] = time.time()
            _state[1] = -1
            world.add_update_trigger(trigger)
            yield From(world.pause(False))

            while _state[1] < 0:
                # Wait until the simulation time has passed the required
                # number of seconds.
                yield From(trollius.sleep(0.1))

            print("%d\t%d\t%f\t%f" % (n, i, _state[1], _state[0]))
            yield From(world.pause())
            yield From(yield_wait(world.delete_all_robots()))

            # Sleep to process all old messages
            yield From(trollius.sleep(0.5))
            yield From(world.reset())


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

