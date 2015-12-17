# Offline evolution scheme
# - We use a population of constant size 10
# - Each robot is evaluated for 20 seconds, though we may vary this number
# - The average speed during this evaluation is the fitness
# - We do parent selection using a binary tournament: select two parents at
#   random, the one with the best fitness is parent 1, repeat for parent 2.
# - Using this mechanism, we generate 10 children
# - After evaluation of the children, we either do:
# -- Plus scheme, sort *all* robots by fitness
# -- Comma scheme, get rid of the parents and continue with children only
from __future__ import absolute_import
import sys
import os

# Add "tol" directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

import trollius
from trollius import From, Return
from tol.config import Config
from tol.manage import World
from sdfbuilder import Pose
from sdfbuilder.math import Vector3
import random
import csv
import itertools
from tol.logging import logger, output_console
import logging

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Script configuration
# Number of individuals in the population
POPULATION_SIZE = 10

# Number of children created in each generation
NUM_CHILDREN = 10

# Keep the parents or only use children
KEEP_PARENTS = True

# Number of generations to produce
NUM_GENERATIONS = 80

# Number of simulation seconds each individual is evaluated
EVALUATION_TIME = 8

# The number of seconds we ignore the robot initially, this allows it
# to i.e. topple over when put down without that being counted as movement
WARMUP_TIME = 3

# Number of times per second we request the world to give
# us a model pose update
POSE_UPDATE_FREQUENCY = 50

# Maximum number of mating attempts between two parents
MAX_MATING_ATTEMPTS = 5


@trollius.coroutine
def evaluate_pair(world, tree, bbox):
    """
    Evaluates a single robot tree.
    :param world:
    :type world: World
    :param tree:
    :param bbox:
    :return: Evaluated Robot object
    """
    # Pause the world just in case it wasn't already
    yield From(world.pause(True))

    pose = Pose(position=Vector3(0, 0, -bbox.min.z))
    fut = yield From(world.insert_robot(tree, pose))
    robot = yield From(fut)

    # Unpause the world to start evaluation
    yield From(world.pause(False))

    while True:
        if robot.age() >= (EVALUATION_TIME + WARMUP_TIME):
            break

        yield From(trollius.sleep(0.01))

    fut = yield From(world.delete_robot(robot))
    yield From(world.pause(True))
    yield From(fut)
    raise Return(robot)


@trollius.coroutine
def evaluate_population(world, trees, bboxes):
    """
    :param world:
    :param trees:
    :param bboxes:
    :return:
    """
    robots = []
    print("Evaluating population...")
    for tree, bbox in itertools.izip(trees, bboxes):
        print("Evaluating individual...")
        robot = yield From(evaluate_pair(world, tree, bbox))
        robots.append(robot)
        print("Done.")

    print("Done evaluating population.")
    raise Return(robots)


def select_parent(parents):
    """
    Select a parent using a binary tournament.
    :param parents:
    :return:
    """
    p = random.sample(parents, 2)
    return p[0] if p[0].velocity() > p[1].velocity() else p[1]


def select_parents(parents):
    """
    :param parents:
    :return:
    """
    p1 = select_parent(parents)
    p2 = p1
    while p2 == p1:
        p2 = select_parent(parents)

    return p1, p2


@trollius.coroutine
def produce_generation(world, parents):
    """
    Produce the next generation of robots from
    the current.
    :param world:
    :param parents:
    :return:
    """
    print("Producing generation...")
    trees = []
    bboxes = []

    while len(trees) < NUM_CHILDREN:
        print("Producing individual...")
        p1, p2 = select_parents(parents)

        for j in xrange(MAX_MATING_ATTEMPTS):
            pair = yield From(world.attempt_mate(p1, p2))
            if pair:
                trees.append(pair[0])
                bboxes.append(pair[1])
                break
        print("Done.")

    print("Done producing generation.")
    raise Return(trees, bboxes)


def log_generation(gen_out, evo, generation, robots):
    """
    :param evo: The evolution run
    :param generation:
    :param gen_out:
    :param robots:
    :return:
    """
    print("================== GENERATION %d ==================" % generation)
    for robot in robots:
        gen_out.writerow([evo, generation, robot.robot.id, robot.velocity()])


@trollius.coroutine
def run(num_evolutions=10):
    """

    :param num_evolutions: The number of times to run the entire evolution
    :return:
    """
    conf = Config(
        output_directory='output',

        # A convenient way to take only the eval time seconds
        # into account is by making that the size of the
        # speed window.
        speed_window=EVALUATION_TIME * POSE_UPDATE_FREQUENCY,
        enable_touch_sensor=True,
        enable_light_sensor=False
    )

    world = yield From(World.create(conf))

    fut = yield From(world.set_pose_update_frequency(POSE_UPDATE_FREQUENCY))
    yield From(fut)

    # Open generations file line buffered
    gen_file = open(os.path.join(world.output_directory, 'generations.csv'), 'wb', buffering=1)
    gen_out = csv.writer(gen_file, delimiter=',')
    gen_out.writerow(['run', 'gen', 'robot_id', 'vel'])

    for evo in range(1, num_evolutions + 1):
        trees, bboxes = yield From(world.generate_population(POPULATION_SIZE))
        robots = yield From(evaluate_population(world, trees, bboxes))
        log_generation(gen_out, evo, 0, robots)

        for generation in xrange(1, NUM_GENERATIONS):
            # Produce the next generation and evaluate them
            child_trees, child_bboxes = yield From(produce_generation(world, robots))
            children = yield From(evaluate_population(world, child_trees, child_bboxes))

            if KEEP_PARENTS:
                robots = children + robots
            else:
                robots = children

            # Sort the bots and reduce to population size
            robots = sorted(robots, key=lambda r: r.velocity(), reverse=True)[:POPULATION_SIZE]
            log_generation(gen_out, evo, generation, robots)

    gen_file.close()
    yield From(world.teardown())


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()
