from __future__ import absolute_import
import random
import itertools
from revolve.build.util import in_cm, in_mm
from revolve.util import Time
import os
import sys
import logging
import argparse

# Add "tol" directory to Python path
from pygazebo.pygazebo import DisconnectError
from trollius.py33_exceptions import ConnectionResetError, ConnectionRefusedError

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return, Future
from pygazebo.msg.request_pb2 import Request

# Revolve / sdfbuilder
from sdfbuilder.math import Vector3
from sdfbuilder import Pose, Model, Link, SDF

# ToL
from tol.config import Config, constants
from tol.manage import World
from tol.logging import logger, output_console
from tol.util import multi_future

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Good seeds so far: 642735, 241276, 939768, 872168
# Not recorded:
# !320135
# 247934
# 4974686
# 4625075
# 541073
# 582708
# 55887, 667758, 989959, 976318

parser = argparse.ArgumentParser(description="Run the Nemo Demo")
parser.add_argument("-s", "--seed", default=-1, help="Supply a random seed", type=int)
parser.add_argument("-n", "--num-initial-bots", default=3,
                    help="Number of initial bots", type=int)
parser.add_argument("-f", "--fast", help="Short reproduction wait.",
                    action="store_true")
parser.add_argument("-i", "--interactive", action="store_true",
                    help="Enable interactive mode (no automatic reproduction)")

parent_color = (1, 0, 0, 0.5)
child_color = (0, 1, 0, 0.5)
insert_z = 0.5


@trollius.coroutine
def create_highlights(world, pa, pb, pc):
    """

    :param world:
    :param pa:
    :type pa: Vector3
    :param pb:
    :type pb: Vector3
    :param pc:
    :type pc: Vector3
    :return:
    """
    fut1, ha = yield From(world.add_highlight(pa, parent_color))
    fut2, hb = yield From(world.add_highlight(pb, parent_color))
    fut3, hc = yield From(world.add_highlight(pc, child_color))

    yield From(multi_future([fut1, fut2, fut3]))
    raise Return(ha, hb, hc)


@trollius.coroutine
def remove_highlights(world, hls):
    """

    :param world:
    :return:
    """
    futures = []
    for hl in hls:
        fut = yield From(world.delete_model(hl.name))
        futures.append(fut)

    yield From(multi_future(futures))


def pick_position(conf):
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
    return Vector3(x, y, insert_z)


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
def do_mate(world, ra, rb):
    """

    :param world:
    :param ra:
    :param rb:
    :return:
    """
    mate = None
    while True:
        # Attempt reproduction
        mate = yield From(world.attempt_mate(ra, rb))

        if mate:
            break

    logger.debug("Mates selected, highlighting points...")

    new_pos = get_insert_position(world.conf, ra, rb, world)
    new_pos.z = insert_z
    hls = yield From(create_highlights(
        world, ra.last_position, rb.last_position, new_pos))
    yield From(trollius.sleep(2))

    logger.debug("Inserting child...")
    child, bbox = mate
    pose = Pose(position=new_pos)
    future = yield From(world.insert_robot(child, pose, parents=[ra, rb]))
    yield From(future)

    yield From(trollius.sleep(2))

    logger.debug("Removing highlights...")
    yield From(remove_highlights(world, hls))


@trollius.coroutine
def interactive_mode(args, world):
    """

    :param args:
    :param world:
    :type world: World
    :return:
    """
    reproduce = []

    def callback(data):
        req = Request()
        req.ParseFromString(data)
        if req.request == "produce_offspring":
            reproduce.append(req.data.split("+++"))

    subscriber = world.manager.subscribe(
        '/gazebo/default/request', 'gazebo.msgs.Request', callback)
    yield From(subscriber.wait_for_connection())

    while True:
        yield From(trollius.sleep(0.1))
        while len(reproduce):
            p1, p2 = reproduce.pop(0)
            ra = world.get_robot_by_name(p1)
            rb = world.get_robot_by_name(p2)

            if not ra or not rb:
                print("Cannot find both parent robots `%s` and `%s`, skipping." % (p1, p2))
                continue

            yield From(do_mate(world, ra, rb))


@trollius.coroutine
def automatic_mode(args, world):
    """

    :param args:
    :param world:
    :return:
    """
    while True:
        logger.debug("Simulating (make sure the world is running)...")
        yield From(sleep_sim_time(world, 5 if args.fast else 15))

        robots = world.robot_list()
        ra, rb = random.sample(robots, 2)
        yield From(do_mate(world, ra, rb))


@trollius.coroutine
def run_server(args):
    """

    :param args:
    :return:
    """
    conf = Config(
        proposal_threshold=0,
        output_directory=None,
        arena_size=(3, 3),
        min_parts=6,
        max_parts=15
    )

    world = yield From(World.create(conf))
    yield From(world.pause(True))

    start_bots = args.num_initial_bots
    poses = [Pose(position=pick_position(conf)) for _ in range(start_bots)]
    trees, bboxes = yield From(world.generate_population(len(poses)))
    fut = yield From(world.insert_population(trees, poses))
    yield From(fut)

    # --- Manual (interactive) reproduction
    if args.interactive:
        print("Interactive mode enabled.")
        yield From(interactive_mode(args, world))
    else:
        yield From(automatic_mode(args, world))


def main():
    args = parser.parse_args()
    seed = random.randint(1, 1000000) if args.seed < 0 else args.seed
    random.seed(seed)
    print("Seed: %d" % seed)

    def handler(loop, context):
        exc = context['exception']
        if isinstance(exc, DisconnectError) or isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)

        raise context['exception']

    try:
        loop = trollius.get_event_loop()
        loop.set_exception_handler(handler)
        loop.run_until_complete(run_server(args))
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")
    except ConnectionRefusedError:
        print("Connection refused, are the world and the analyzer loaded?")

if __name__ == '__main__':
    main()

