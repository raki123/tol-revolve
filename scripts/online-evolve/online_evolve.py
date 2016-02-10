"""
Online evolution experiment. Initially we try to answer the following questions:

- How do complexity and size evolve?
- How does this depend on `X` (variable to be determined)
- How can we make the system stable? Depending on # of babies,
  initial / max population size, age of death.
"""
import csv
import logging

import itertools

import math
import random

import time
from sdfbuilder import Pose
from sdfbuilder.math import Vector3

import os
import shutil

import trollius
from trollius import From, Return
from revolve.util import multi_future, wait_for, Time
from tol.config import parser
from tol.manage import World
from tol.logging import logger, output_console

# Output to console and enable debug logging
output_console()
logger.setLevel(logging.DEBUG)

# Environment parameters
parser.add_argument(
    '--world-diameter',
    default=50, type=float,
    help="The diameter of the environment in meters."
)

parser.add_argument(
    '--num-wall-segments',
    default=12, type=int,
    help="The number of segments the arena wall will consist off."
)

parser.add_argument(
    '--birth-clinic-diameter',
    default=3.0, type=float,
    help="The diameter of the birth clinic in meters."
)

parser.add_argument(
    '--min-drop-distance',
    default=0.25, type=float,
    help="Minimum distance in meters to the other robots when being dropped "
         "into the world. This cannot always be guaranteed, so a number of "
         "attempts is made."
)

# General population parameters
parser.add_argument(
    '--initial-population-size',
    default=15, type=int,
    help="The size of the starting population."
)

parser.add_argument(
    '--max-population-size',
    default=60, type=int,
    help="The maximum size of the population."
)

parser.add_argument(
    '--max-lifetime',
    default=36000, type=float,
    help="The absolute maximum number of seconds a robot is"
         " allowed to live."
)

parser.add_argument(
    '--initial-age-mu',
    default=36000 / 6.0, type=float,
    help="Gaussian mean for the age distribution of the initial population."
)

parser.add_argument(
    '--initial-age-sigma',
    default=36000 / 12.0, type=float,
    help="Gaussian standard deviation for age distribution of "
         "the initial population."
)

parser.add_argument(
    '--age-cutoff',
    default=0.05, type=float,
    help="A robot's age of death is determined by the formula "
         "`Ml * min(0.5 * (f1 + f2), c)/c`, where `Ml` is the maximum lifetime, "
         "`f1` and `f2` are the robot parents' fitness values and"
         " `c` is this cutoff value. "
         "This results in a linear increase between zero and the maximum "
         "age."
)

# Mating parameters
parser.add_argument(
    '--mating-distance-threshold',
    default=1.0, type=float,
    help="The mating distance threshold in meters."
)

parser.add_argument(
    '--mating-fitness-threshold',
    default=0.3, type=float,
    help="The maximum fractional fitness difference between two robots that "
         "will allow a mate. E.g. for a fraction of 0.5, two robots will not mate"
         " if one is 50% less fit than the other."
)

parser.add_argument(
    '--gestation-period',
    default=36000 / 100.0, type=float,
    help="The minimum time a robot has to wait between matings."
)

parser.add_argument(
    '--max-pair-children',
    default=4, type=int,
    help="The maximum number of children one pair of robots is allowed to have."
)

# Experiment parameters
parser.add_argument(
    '--num-repetitions',
    default=1, type=int,
    help="The number of times to repeat the experiment."
)

parser.add_argument(
    '--explosion-cutoff',
    default=45, type=int,
    help="The number of robots in the world beyond which the experiment"
         " result is set to `explosion`."
)

parser.add_argument(
    '--extinction-cutoff',
    default=3, type=int,
    help="The number of robots in the world beyond which the experiment"
         " result is set to `extinction`."
)

parser.add_argument(
    '--stability-cutoff',
    default=7200, type=float,
    help="The number of simulation seconds after which the experiment"
         " result is set to `extinction`."
)


class OnlineEvoManager(World):
    """
    World manager extended with capabilities for online evolution.
    """

    def __init__(self, conf, _private):
        """

        :param conf:
        :param _private:
        :return:
        """
        super(OnlineEvoManager, self).__init__(conf, _private)

        self.fitness_filename = None
        self.fitness_file = None
        self.write_fitness = None

        if self.do_restore:
            self.current_run = self.do_restore['current_run']
        else:
            self.current_run = 0

        if self.output_directory:
            self.fitness_filename = os.path.join(self.output_directory, 'fitness.csv')

            if self.do_restore:
                shutil.copy(self.fitness_filename + '.snapshot', self.fitness_filename)
                self.fitness_file = open(self.fitness_filename, 'ab', buffering=1)
                self.write_fitness = csv.writer(self.fitness_file, delimiter=',')
            else:
                self.fitness_file = open(self.fitness_filename, 'wb', buffering=1)
                self.write_fitness = csv.writer(self.fitness_file, delimiter=',')
                self.write_fitness.writerow(['run', 't_sim', 'robot_id', 'age', 'displacement',
                                             'vel', 'dvel', 'fitness'])

    @classmethod
    @trollius.coroutine
    def create(cls, conf):
        """
        Coroutine to instantiate a Revolve.Angle WorldManager
        :param conf:
        :return:
        """
        self = cls(_private=cls._PRIVATE, conf=conf)
        yield From(self._init())
        raise Return(self)

    @trollius.coroutine
    def teardown(self):
        """

        :return:
        """
        yield From(super(OnlineEvoManager, self).teardown())
        if self.fitness_file:
            self.fitness_file.close()

    @trollius.coroutine
    def get_snapshot_data(self):
        """
        :return:
        """
        data = yield From(super(OnlineEvoManager, self).get_snapshot_data())
        data['current_run'] = self.current_run
        raise Return(data)

    @trollius.coroutine
    def create_snapshot(self):
        """
        Copy the fitness file in the snapshot
        :return:
        """
        ret = yield From(super(OnlineEvoManager, self).create_snapshot())
        if not ret:
            raise Return(ret)

        self.fitness_file.flush()
        shutil.copy(self.fitness_filename, self.fitness_filename + '.snapshot')

    @trollius.coroutine
    def build_arena(self):
        """
        Initializes the arena by building square arena wall blocks.
        :return: Future that resolves when arena building is done.
        """
        logger.debug("Building the arena...")
        n = self.conf.num_wall_segments

        r = self.conf.world_diameter * 0.5
        frac = 2 * math.pi / n
        points = [Vector3(r * math.cos(i * frac), r * math.sin(i * frac), 0) for i in range(n)]
        fut = yield From(self.build_walls(points))
        raise Return(fut)

    def select_mates(self):
        """
        Finds all mate combinations in the current arena.
        :return:
        """
        robot_list = self.robots.values()
        return [(ra, rb) for ra, rb in itertools.combinations(robot_list, 2)
                if ra.will_mate_with(rb) and rb.will_mate_with(ra)]

    @trollius.coroutine
    def birth(self, tree, bbox, parents):
        """
        Birth process, picks a robot position and inserts
        the robot into the world.
        :param tree:
        :param bbox:
        :param parents:
        :return:
        """
        r = self.conf.birth_clinic_diameter / 2.0

        # Drop height is 20cm here
        pos = None
        done = False
        z = -bbox.min.z + 0.2
        for _ in xrange(5):
            # Make 5 attempts, if we still don't have a satisfactory position
            # just use the last one.
            angle = random.random() * 2 * math.pi
            pos = Vector3(r * math.cos(angle), r * math.sin(angle), z)
            done = True

            for bot in self.robots.values():
                if bot.distance_to(pos) < self.conf.min_drop_distance:
                    done = False
                    break

            if done:
                break

        if not done:
            logger.warning("Warning: could not satisfy minimal drop distance.")

        fut = yield From(self.insert_robot(tree, Pose(position=pos), parents))
        raise Return(fut)

    @trollius.coroutine
    def kill_old_robots(self):
        """
        Kills old robots, returns a list of futures of all
        delete requests (robots should be deleted once these
        are resolved).
        :return:
        """
        futs = []
        for robot in self.robot_list():
            if robot.age() > robot.age_of_death:
                fut = yield From(self.delete_robot(robot))
                futs.append(fut)

        raise Return(futs)

    def log_fitness(self):
        """
        :return:
        """
        if not self.write_fitness:
            return

        t = float(self.last_time)
        n = self.current_run
        for robot in self.robots.values():
            ds, dt = robot.displacement()
            self.write_fitness.writerow([n, t, robot.robot.id,
                                         robot.age(), ds.norm(), robot.velocity(),
                                         robot.displacement_velocity(), robot.fitness()])

    @trollius.coroutine
    def run(self):
        """
        Simple wrapper to complete multiple simulation runs.
        :param n:
        :return:
        """
        while self.current_run < self.conf.num_repetitions:
            yield From(self.run_single())

            # Update run counter and clear restore status
            self.current_run += 1
            self.do_restore = None

    @trollius.coroutine
    def run_single(self):
        """
        :return:
        """
        conf = self.conf
        insert_queue = []

        if not self.do_restore:
            if self.current_run == 0:
                # Only build arena on first run
                yield From(wait_for(self.build_arena()))

            # Generate a starting population
            trees, bboxes = yield From(self.generate_population(conf.initial_population_size))
            insert_queue = zip(trees, bboxes, [None for _ in range(len(trees))])

        # Simple loop timing mechanism
        timers = {k: Time() for k in ['reproduce', 'death', 'snapshot',
                                      'log_fitness', 'rtf']}
        this = self

        def timer(name, t):
            """
            :param t:
            :param name:
            :return:
            """
            if this.last_time is not None and float(this.last_time - timers[name]) > t:
                timers[name] = this.last_time
                return True

            return False

        # Some variables
        real_time = time.time()
        rtf_interval = 2.5
        sleep_time = 0.1
        run_result = 'unknown'

        # Start the world
        yield From(wait_for(self.pause(False)))
        while True:
            if insert_queue:
                tree, bbox, parents = insert_queue.pop()
                yield From(wait_for(self.birth(tree, bbox, parents)))

            # Perform operations only if there are no items
            # in the insert queue, makes snapshotting easier.
            if insert_queue:
                yield From(trollius.sleep(sleep_time))
                continue

            if timer('snapshot', 100.0):
                # Snapshot the world every 100 simulation seconds
                yield From(self.create_snapshot())
                yield From(wait_for(self.pause(False)))

            if timer('death', 5.0):
                # Kill off robots over their age every 5 simulation seconds
                futs = yield From(self.kill_old_robots())
                if futs:
                    yield From(multi_future(futs))

            if timer('reproduce', 3.0) and len(self.robots) < conf.max_population_size:
                # Attempt a reproduction every 3 simulation seconds
                potential_mates = self.select_mates()
                if potential_mates:
                    ra, rb = random.choice(potential_mates)
                    result = yield From(self.attempt_mate(ra, rb))

                    if result:
                        ra.did_mate_with(rb)
                        rb.did_mate_with(ra)
                        child, bbox = result
                        insert_queue.append((child, bbox, (ra, rb)))

            if timer('log_fitness', 2.0):
                # Log overall fitness every 2 simulation seconds
                self.log_fitness()

            if timer('rtf', rtf_interval):
                # Print RTF to screen every so often
                nw = time.time()
                diff = nw - real_time
                real_time = nw
                print("RTF: %f" % (rtf_interval / diff))

            # Stop conditions
            num_bots = len(self.robots)
            if num_bots <= conf.extinction_cutoff:
                print("%d or fewer robots left in population - extinction." %
                      conf.extinction_cutoff)
                run_result = 'extinction'
                break
            elif num_bots >= conf.explosion_cutoff:
                print("%d or more robots in population - explosion" %
                      conf.explosion_cutoff)
                run_result = 'explosion'
                break
            elif float(self.age()) > conf.stability_cutoff:
                print("World older than %f seconds, stable." % conf.stability_cutoff)
                run_result = 'stable'
                break

            yield From(trollius.sleep(sleep_time))

        # Delete all robots and reset the world, just in case a new run
        # will be started.
        yield From(wait_for(self.delete_all_robots()))
        yield From(wait_for(self.reset()))
        yield From(wait_for(self.pause(True)))
        self.write_result(run_result)

    def write_result(self, result):
        """
        Writes the textual result status of a single run
        :param result:
        :return:
        """
        if self.output_directory:
            result_filename = os.path.join(self.output_directory, 'results.csv')
            exists = os.path.exists(result_filename)

            with open(result_filename, 'ab') as f:
                writer = csv.writer(f, delimiter=',')
                if not exists:
                    writer.writerow(['run', 'result'])

                writer.writerow([self.current_run, result])


@trollius.coroutine
def run():
    """
    :return:
    """
    conf = parser.parse_args()
    world = yield From(OnlineEvoManager.create(conf))
    yield From(world.run())
    yield From(world.teardown())


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()
