from __future__ import absolute_import
import random
import itertools
from revolve.build.util import in_cm, in_mm
from revolve.util import Time
import os
import sys
import logging

# Add "tol" directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3
from sdfbuilder import Pose, Model, Link, SDF

# ToL
from tol.config import Config, constants
from tol.manage import World
from tol.logging import logger, output_console

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Set command line seed if supplied, otherwise choose a random number
# Good seeds so far: 642735, 241276, 939768
# Not recorded:
# 4974686
# 4625075
# 541073
# 582708
# 55887, 667758, 989959, 976318
provided_seed = len(sys.argv) > 1
if provided_seed:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 10000000)

random.seed(seed)
print("Seed: %d" % seed)


class Highlight(Link):
    """
    Model to highlight newly inserted robots / selected parents
    """

    def __init__(self, name, color, **kwargs):
        super(Highlight, self).__init__(name, **kwargs)
        self.make_cylinder(10e10, in_cm(40), in_mm(1), collision=False)
        r, g, b, a = color
        self.make_color(r, g, b, a)


@trollius.coroutine
def create_highlights(world, pa, pb, pc, counter):
    """

    :param world:
    :param pa:
    :type pa: Vector3
    :param pb:
    :type pb: Vector3
    :param pc:
    :type pc: Vector3
    :param counter:
    :return:
    """
    parent_color = (1, 0, 0, 0.5)
    child_color = (0, 1, 0, 0.5)
    ha = Highlight("pa", parent_color)
    hb = Highlight("pb", parent_color)
    hc = Highlight("pc", child_color)

    pa, pb, pc = pa.copy(), pb.copy(), pc.copy()
    pa.z = pb.z = pc.z = 0
    ha.set_position(pa)
    hb.set_position(pb)
    hc.set_position(pc)

    model = Model(name="highlights_%d" % counter, static=True, elements=[ha, hb, hc])
    sdf = SDF(elements=[model])
    fut = yield From(world.insert_model(sdf))
    yield From(fut)


@trollius.coroutine
def remove_highlights(world, counter):
    """

    :param world:
    :param counter:
    :return:
    """
    fut = yield From(world.delete_model("highlights_%d" % counter))
    yield From(fut)


def pick_position(conf, z=0.0):
    """

    :param conf:
    :param z: z height of the returned vector
    :return:
    """
    margin = in_cm(20)
    x_min, x_max = -0.5 * conf.arena_size[0] + margin, 0.5 * conf.arena_size[0] - margin
    y_min, y_max = -0.5 * conf.arena_size[1] + margin, 0.5 * conf.arena_size[1] - margin

    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return Vector3(x, y, z)


@trollius.coroutine
def sleep_sim_time(world, seconds):
    """
    Sleeps for a certain number of simulation seconds,
    note that this is always approximate as it relies
    on real world sleeping.
    :param world:
    :param seconds:
    :return:
    """
    start = world.last_time if world.last_time else Time()
    remain = seconds

    while True:
        # Sleep for 0.1 seconds, but never less than 0.05
        yield_for = max(0.05, 0.1 * remain)
        yield From(trollius.sleep(yield_for))
        now = world.last_time if world.last_time else Time()
        remain = seconds - float(now - start)

        if remain <= 0:
            break


def get_insert_position(conf, ra, rb, world):
    """
    Tries to find a suitable insert position

    :param conf:
    :param ra:
    :param rb:
    :param world:
    :return:
    """
    good = False
    new_pos = None

    while not good:
        # Get position on the line separating the two robots
        new_pos = world.get_equilateral_position(ra, rb, random.uniform(0, 2.5))

        # Add some randomness to the insert position
        new_pos += 0.1 * pick_position(conf)

        good = True
        for r in world.robots.values():
            if not r.last_position:
                continue

            # Only choose positions that are more than 25cm
            # away from the nearest bot but closer than 4m
            # from the furthest bot.
            diff = r.last_position - new_pos
            dist = diff.norm()
            if dist < in_cm(25) or dist > 4:
                good = False
                break

    return new_pos


@trollius.coroutine
def run_server():
    conf = Config(
        proposal_threshold=0,
        output_directory='output',
        arena_size=(3, 3),
        min_parts=6,
        max_parts=15
    )

    # Height to drop new robots from
    insert_z = 0.5

    # Highlight counter
    hl_count = 0

    world = yield From(World.create(conf))
    yield From(world.pause(True))

    start_bots = 3
    poses = [Pose(position=pick_position(conf, insert_z)) for _ in range(start_bots)]
    trees, bboxes = yield From(world.generate_population(len(poses)))
    fut = yield From(world.insert_population(trees, poses))
    yield From(fut)

    while True:
        logger.debug("Simulating (make sure the world is running)...")
        yield From(sleep_sim_time(world, 15 if provided_seed else 5))

        mate = ra = rb = None
        while True:
            # Attempt reproduction
            logger.debug("Attempting mate selection...")
            robots = world.robots.values()
            ra, rb = random.sample(robots, 2)
            mate = yield From(world.attempt_mate(ra, rb))

            if mate:
                break

            logger.debug("Attempt failed.")

        logger.debug("Found mates! Highlighting points...")

        new_pos = get_insert_position(conf, ra, rb, world)
        new_pos.z = insert_z
        yield From(create_highlights(world, ra.last_position, rb.last_position,
                                     new_pos, hl_count))
        yield From(trollius.sleep(2.0))

        logger.debug("Inserting child...")
        child, bbox = mate
        pose = Pose(position=new_pos)
        future = yield From(world.insert_robot(child, pose, parents=[ra, rb]))
        yield From(future)

        yield From(trollius.sleep(3))

        logger.debug("Removing highlights...")
        yield From(remove_highlights(world, hl_count))
        hl_count += 1


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")

if __name__ == '__main__':
    main()

