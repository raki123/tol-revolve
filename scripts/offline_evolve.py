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
from revolve.util import wait_for
import trollius
from trollius import From, Return
from sdfbuilder import Pose
from sdfbuilder.math import Vector3
import random
import csv
import itertools
import logging

# Add "tol" directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from tol.config import parser
from tol.manage import World
from tol.logging import logger, output_console

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Add offline evolve arguments
parser.add_argument(
    '--population-size',
    default=10, type=int,
    help="Population size in each generation."
)

parser.add_argument(
    '--num-children',
    default=10, type=int,
    help="The number of children produced in each generation."
)

parser.add_argument(
    '--keep-parents',
    default=True, type=lambda v: v.lower() == "true" or v == "1",
    help="Whether or not to discard the parents after each generation. This determines the strategy, + or ,."
)

parser.add_argument(
    '--num-generations',
    default=80, type=int,
    help="The number of generations to simulate."
)

parser.add_argument(
    '--evaluation-time',
    default=8, type=float,
    help="The number of simulation seconds each individual is evaluated."
)

parser.add_argument(
    '--warmup-time',
    default=3, type=float,
    help="The number of seconds the robot is initially ignored, allows it to e.g. topple over"
         " when put down without that being counted as movement."
)

parser.add_argument(
    '--max-mating-attempts',
    default=5, type=int,
    help="Maximum number of mating attempts between two parents."
)


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

    max_age = world.conf.evaluation_time + world.conf.warmup_time

    # Unpause the world to start evaluation
    yield From(world.pause(False))

    while True:
        if robot.age() >= max_age:
            break

        # Sleep for the pose update frequency, which is about when
        # we expect a new age update.
        yield From(trollius.sleep(1.0 / world.pose_update_frequency))

    yield From(wait_for(world.delete_robot(robot)))
    yield From(wait_for(world.pause(True)))
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

    while len(trees) < world.conf.num_children:
        print("Producing individual...")
        p1, p2 = select_parents(parents)

        for j in xrange(world.conf.max_mating_attempts):
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
    conf = parser.parse_args()
    world = yield From(World.create(conf))

    # Open generations file line buffered
    gen_file = open(os.path.join(world.output_directory, 'generations.csv'), 'wb', buffering=1)
    gen_out = csv.writer(gen_file, delimiter=',')
    gen_out.writerow(['run', 'gen', 'robot_id', 'vel'])

    for evo in range(1, num_evolutions + 1):
        trees, bboxes = yield From(world.generate_population(conf.population_size))
        robots = yield From(evaluate_population(world, trees, bboxes))
        log_generation(gen_out, evo, 0, robots)

        for generation in xrange(1, conf.num_generations):
            # Produce the next generation and evaluate them
            child_trees, child_bboxes = yield From(produce_generation(world, robots))
            children = yield From(evaluate_population(world, child_trees, child_bboxes))

            if conf.keep_parents:
                robots = children + robots
            else:
                robots = children

            # Sort the bots and reduce to population size
            robots = sorted(robots, key=lambda r: r.velocity(), reverse=True)[:conf.population_size]
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
