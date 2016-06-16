"""
Online evolution experiment. Initially we try to answer the following questions:

- How do complexity and size evolve?
- How does this depend on `X` (variable to be determined)
- How can we make the system stable? Depending on # of babies,
  initial / max population size, age of death.
"""
import csv
import logging
import sys

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

# ToL imports may require the system path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

from tol.manage.robot import Robot
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
    '--drop-height',
    default=0.25, type=float,
    help="The height from which robots are dropped into the world."
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
    '--population-size',
    default=15, type=int,
    help="The minimum size of the maintained population."
)

# Mating parameters
parser.add_argument(
    '--mating-distance-threshold',
    default=50.0, type=float,
    help="The mating distance threshold in meters."
)

parser.add_argument(
    '--mating-fitness-threshold',
    default=0.5, type=float,
    help="The maximum fractional fitness difference between two robots that "
         "will allow a mate. E.g. for a fraction of 0.5, two robots will not mate"
         " if one is 50%% less fit than the other."
)

# Experiment parameters
parser.add_argument(
    '--num-repetitions',
    default=30, type=int,
    help="The number of times to repeat the experiment."
)

parser.add_argument(
    '--birth-limit',
    default=15 * 200, type=int,
    help="The number of evaluated births after which to stop the experiment."
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

        # Output files
        csvs = {
            'fitness': ['run', 't_sim', 'births', 'robot_id', 'age', 'displacement',
                        'vel', 'dvel', 'fitness'],
            'summary': ['run', 'world_age', 'robot_count', 'part_count',
                        'births', 'deaths'],
            'deaths': ['run', 'world_age', 'robot_id', 'x', 'y', 'z']
        }
        self.csv_files = {k: {'filename': None, 'file': None, 'csv': None,
                              'header': csvs[k]}
                          for k in csvs}

        data = self.do_restore
        if self.do_restore:
            self.current_run = data['current_run']
            self.births = data['births']
            self.deaths = data['deaths']
        else:
            self.current_run = 0
            self.births = 0
            self.deaths = 0

        if self.output_directory:
            for k in self.csv_files:
                fname = os.path.join(self.output_directory, k+'.csv')
                self.csv_files[k]['filename'] = fname
                if self.do_restore:
                    shutil.copy(fname + '.snapshot', fname)
                    f = open(fname, 'ab', buffering=1)
                else:
                    f = open(fname, 'wb', buffering=1)

                self.csv_files[k]['file'] = f
                self.csv_files[k]['csv'] = csv.writer(f, delimiter=',')

                if not self.do_restore:
                    self.csv_files[k]['csv'].writerow(self.csv_files[k]['header'])

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

        for k in self.csv_files:
            if self.csv_files[k]['file']:
                self.csv_files[k]['file'].close()

    @trollius.coroutine
    def get_snapshot_data(self):
        """
        :return:
        """
        data = yield From(super(OnlineEvoManager, self).get_snapshot_data())
        data.update({
            'current_run': self.current_run,
            'births': self.births,
            'deaths': self.deaths
        })
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

        for k in self.csv_files:
            entry = self.csv_files[k]
            if entry['file']:
                entry['file'].flush()
                shutil.copy(entry['filename'], entry['filename'] + '.snapshot')

    @trollius.coroutine
    def build_arena(self):
        """
        Initializes the arena by building square arena wall blocks.
        :return: Future that resolves when arena building is done.
        """
        logger.debug("Building the arena...")
        n = self.conf.num_wall_segments

        futs = []
        r = self.conf.world_diameter * 0.5
        frac = 2 * math.pi / n
        points = [Vector3(r * math.cos(i * frac), r * math.sin(i * frac), 0) for i in range(n)]
        fut = yield From(self.build_walls(points))
        futs.append(fut)
        raise Return(multi_future(futs))

    def select_parent(self, parents):
        """

        :return:
        """
        return sorted(random.sample(parents, self.conf.tournament_size),
                      key=lambda r: r.fitness())[-1]

    def select_parents(self):
        """
        Returns a list of robots that have at least one potential mate.

        :return:
        """
        parents = self.robot_list()
        p1 = self.select_parent(parents)
        p2 = self.select_parent(list(parent for parent in parents if parent != p1))
        return p1, p2

    @trollius.coroutine
    def birth(self, tree, bbox, parents):
        """
        Birth process, picks a robot position and inserts
        the robot into the world.

        Robots are currently placed at a random position within the circular
        birth clinic. In this process, 5 attempts are made to place the robot
        at the minimum drop distance from other robots. If this fails however
        the last generated position is used anyway.
        :param tree:
        :param bbox:
        :param parents:
        :return:
        """
        # Pick a random radius and angle within the birth clinic
        pos = Vector3()
        pos.z = -bbox.min.z + self.conf.drop_height
        done = False
        min_drop = self.conf.min_drop_distance

        for i in range(5):
            angle = random.random() * 2 * math.pi
            radius = self.conf.birth_clinic_diameter * 0.5 * random.random()
            pos.x = radius * math.cos(angle)
            pos.y = radius * math.sin(angle)

            done = True
            for robot in self.robots.values():
                if robot.distance_to(pos, planar=True) < min_drop:
                    done = False
                    break

            if done:
                break

        if not done:
            logger.warning("Warning: could not satisfy minimum drop distance.")

        # Note that we register the reproduction only if
        # the child is actually born, i.e. there were enough parts
        # left in the world to satisfy the request.
        if parents:
            ra, rb = parents
            ra.did_mate_with(rb)
            rb.did_mate_with(ra)

        self.births += 1
        fut = yield From(self.insert_robot(tree, Pose(position=pos), parents=parents))
        raise Return(fut)

    @trollius.coroutine
    def kill_robots(self):
        """
        Kills selected robots.
        :return:
        """
        bots = sorted(self.robot_list(), key=lambda r: r.fitness(), reverse=True)
        to_kill = bots[self.conf.population_size:]

        futs = []
        write_deaths = self.csv_files['deaths']['csv']

        for robot in to_kill:
            fut = yield From(self.delete_robot(robot))
            futs.append(fut)

            if write_deaths:
                write_deaths.writerow((self.current_run, self.age(),
                                       robot.robot.id, robot.last_position.x,
                                       robot.last_position.y, robot.last_position.z))

        raise Return(futs)

    @trollius.coroutine
    def produce_generation(self):
        """
        Produce the next generation of robots from
        the current.
        :param parents:
        :return:
        """
        print("Producing generation...")
        trees = []
        bboxes = []
        parent_pairs = []

        while len(trees) < self.conf.population_size:
            print("Producing individual...")
            p1, p2 = self.select_parents()

            for j in xrange(self.conf.max_mating_attempts):
                pair = yield From(self.attempt_mate(p1, p2))
                if pair:
                    trees.append(pair[0])
                    bboxes.append(pair[1])
                    parent_pairs.append((p1, p2))
                    break

            print("Done.")

        print("Done producing generation.")
        raise Return(trees, bboxes, parent_pairs)

    def create_robot_manager(self, robot_name, tree, robot, position, time, battery_level, parents):
        """
        Overriding with robot manager with more capabilities.
        :param robot_name:
        :param tree:
        :param robot:
        :param position:
        :param time:
        :param battery_level:
        :param parents:
        :return:
        """
        return Robot(self.conf, robot_name, tree, robot, position, time, 0.0, parents)

    def total_fitness(self):
        """
        Returns the sum of the fitness of all robots in the system.
        :return:
        """
        return reduce(lambda a, b: a + b.fitness(), self.robots.values(), 0.0)

    def total_size(self):
        """
        Returns the total size of all robots in the system.
        :return:
        """
        return reduce(lambda a, b: a + b.size, self.robots.values(), 0.0)

    def log_fitness(self):
        """
        :return:
        """
        f = self.csv_files['fitness']['csv']
        if not f:
            return

        t = float(self.age())
        n = self.current_run
        for robot in self.robots.values():
            ds, dt = robot.displacement()
            f.writerow([n, t, self.births, robot.robot.id,
                        robot.age(), ds.norm(), robot.velocity(),
                        robot.displacement_velocity(), robot.fitness()])

    def log_summary(self):
        """
        :return:
        """
        f = self.csv_files['summary']['csv']
        if not f:
            return

        f.writerow((self.current_run, self.age(), len(self.robots),
                    self.total_size(), self.births, self.deaths))

    @trollius.coroutine
    def run(self):
        """
        Simple wrapper to complete multiple simulation runs.
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

            # Set initial battery charge
            self.births = 0
            self.deaths = 0

            # Generate a starting population
            trees, bboxes = yield From(self.generate_population(conf.population_size))
            insert_queue = zip(trees, bboxes, [None for _ in range(len(trees))])

        # Simple loop timing mechanism
        timers = {k: Time() for k in ['reproduce', 'death', 'snapshot',
                                      'log_fitness', 'rtf', 'insert_queue']}
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
        rtf_interval = 10.0
        sleep_time = 0.1
        started = False
        t_eval = self.conf.evaluation_time + self.conf.warmup_time

        while True:
            if insert_queue and (not started or timer('insert_queue', 1.0)):
                tree, bbox, parents = insert_queue.pop()
                res = yield From(self.birth(tree, bbox, parents))
                if res:
                    yield From(res)

            if not started:
                # Start the world
                yield From(wait_for(self.pause(False)))
                started = True

            # Perform operations only if there are no items
            # in the insert queue, makes snapshotting easier.
            if insert_queue:
                # Space out robot inserts with one simulation second
                # to allow them to drop in case they are too close.
                # Sleep for a very small interval every time until
                # all inserts are done
                yield From(trollius.sleep(0.01))
                continue

            # Book keeping
            if timer('snapshot', 100.0):
                # Snapshot the world every 100 simulation seconds
                yield From(self.create_snapshot())
                yield From(wait_for(self.pause(False)))

            if timer('log_fitness', 2.0):
                # Log overall fitness every 2 simulation seconds
                self.log_fitness()
                self.log_summary()

            if timer('rtf', rtf_interval):
                # Print RTF to screen every so often
                nw = time.time()
                diff = nw - real_time
                real_time = nw
                print("RTF: %f" % (rtf_interval / diff))

            min_age = min(r.age() for r in self.robot_list())
            if min_age >= t_eval:
                if self.births >= self.conf.birth_limit:
                    # Stop the experiment
                    break

                # - Kill all but the `n` most fit robots
                futs = yield From(self.kill_robots())

                if futs:
                    yield From(multi_future(futs))

                # - Produce `n` children and insert into the world
                trees, bboxes, parent_pairs = yield From(self.produce_generation())
                insert_queue += zip(trees, bboxes, parent_pairs)

            yield From(trollius.sleep(sleep_time))

        # Delete all robots and reset the world, just in case a new run
        # will be started.
        yield From(wait_for(self.delete_all_robots()))
        yield From(wait_for(self.reset()))
        yield From(wait_for(self.pause(True)))
        yield From(trollius.sleep(0.5))


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
