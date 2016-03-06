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
    default=25, type=float,
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
    '--charger-x',
    default=12.0, type=float,
    help="x-coordinate of the position of the charger in the world"
)

parser.add_argument(
    '--charger-y',
    default=12.0, type=float,
    help="y-coordinate of the position of the charger in the world"
)

# Battery charging parameters
parser.add_argument(
    '--charger-r',
    default=0.5, type=float,
    help="Charger degradation distance."
)

parser.add_argument(
    '--total-charge-rate',
    default=150, type=float,
    help="The total number of charge units the charger provides per second. How many of this each "
         "robot receives depends on their number and distance to the charger."
)

parser.add_argument(
    '--max-robot-charge-rate',
    default=15.0, type=float,
    help="The maximum number of charge units a robot battery can increase in one second."
)

parser.add_argument(
    '--max-robot-charge',
    default=7.5 * 36000, type=float,
    help="The maximum robot battery charge in units."
)

parser.add_argument(
    '--initial-charge',
    default=7.5 * 1800, type=float,
    help="Initial battery charge for new robots."
)

# Robot insertion parameters
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
    '--initial-population-size',
    default=12, type=int,
    help="The size of the starting population."
)

parser.add_argument(
    '--part-limit',
    default=500, type=int,
    help="The maximum number of robot parts in the world."
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
    '--extinction-cutoff',
    default=3, type=int,
    help="The number of robots in the world beyond which the experiment"
         " result is set to `extinction`."
)

parser.add_argument(
    '--explosion-cutoff',
    default=60, type=int,
    help="The number of robots in the world beyond which the experiment"
         " result is set to `explosion`."
)

parser.add_argument(
    '--stability-cutoff',
    default=36000, type=float,
    help="The number of simulation seconds after which the experiment"
         " result is set to `stable`."
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
            'fitness': ['run', 't_sim', 'robot_id', 'age', 'displacement',
                        'vel', 'dvel', 'fitness', 'charge', 'size'],
            'summary': ['run', 'world_age', 'robot_count', 'part_count',
                        'births', 'deaths'],
            'deaths': ['run', 'world_age', 'robot_id', 'x', 'y', 'z']
        }
        self.csv_files = {k: {'filename': None, 'file': None, 'csv': None,
                              'header': csvs[k]}
                          for k in csvs}

        self.charger_location = Vector3(self.conf.charger_x, self.conf.charger_y, 0.0)

        data = self.do_restore
        if self.do_restore:
            self.current_run = data['current_run']
            self.last_charge_update = data['last_charge_update']
            self.births = data['births']
            self.deaths = data['deaths']
        else:
            self.last_charge_update = None
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
            'deaths': self.deaths,
            'last_charge_update': self.last_charge_update
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

    def select_parents(self):
        """
        Returns a list of robots that have at least one potential mate.

        :return:
        """
        return [ra for ra, _ in self.select_mates()]

    def select_optimal_mate(self, ra):
        """
        Given a robot, selects the optimal mate. If you call this method
        for a robot without any potential mate you will get an `IndexError`.
        :param ra:
        :type ra: Robot
        :return:
        """
        return sorted([rb for rb in self.robots.values()
                       if ra is not rb and ra.will_mate_with(rb) and rb.will_mate_with(ra)],
                      key=lambda r: r.fitness(), reverse=True)[-1]

    def select_mates(self):
        """
        Finds all mate combinations in the current arena.
        :return:
        """
        return [(ra, rb) for ra, rb in itertools.combinations(self.robots.values(), 2)
                if ra.will_mate_with(rb) and rb.will_mate_with(ra)]

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
        s = len(tree)
        if (s + self.total_size()) > self.conf.part_limit:
            print("Not enough parts in pool to create robot of size %d." % s)
            raise Return(False)

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
        fut = yield From(self.insert_robot(tree, Pose(position=pos), parents=parents,
                                           initial_battery=self.conf.initial_charge))
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
        write_deaths = self.csv_files['deaths']['csv']

        for robot in self.robots.values():
            if robot.get_battery_level() <= 0:
                print("Robot `%s` has an empty battery and will be removed." % robot.name)
                fut = yield From(self.delete_robot(robot))
                futs.append(fut)
                self.deaths += 1

                if write_deaths:
                    write_deaths.writerow((self.current_run, self.age(),
                                           robot.robot.id, robot.last_position.x,
                                           robot.last_position.y, robot.last_position.z))

        raise Return(futs)

    @trollius.coroutine
    def charge_batteries(self):
        """
        :return:
        """
        if self.last_charge_update is None:
            # Only start updating batteries after intial world update
            self.last_charge_update = self.last_time
            return

        t_diff = float(self.last_time - self.last_charge_update)
        self.last_charge_update = self.last_time

        if not self.robots:
            return

        available = self.conf.total_charge_rate
        max_charge_rate = self.conf.max_robot_charge_rate
        max_rate_frac = max_charge_rate / available
        r = self.conf.charger_r

        # First calculate the fraction of the total charge each
        # of these robots could receive theoretically. This is
        # bounded to `max_rate_frac` because a robot will
        # never draw more than this amount.
        # For simplicity we assume that robots which are already
        # maximally charged will still draw charge from the station,
        # but will simply dissipate this as heat.
        max_recv = {}
        recv_sum = 0
        for name, robot in self.robots.iteritems():
            d = robot.distance_to(self.charger_location)
            recv = min(1.0 if d < r else (r / d)**2, max_rate_frac)
            max_recv[name] = recv
            recv_sum += recv

        # If the total charge that can be received by these robots
        # exceeds the total available charge, we apply a correction
        # factor so that the total divided charge will equal the max
        corr = 1.0 if recv_sum < 1.0 else 1.0 / recv_sum

        charge_sum = 0.0
        for name, robot in self.robots.iteritems():
            charge_amount = t_diff * corr * max_recv[name] * available
            charge_sum += charge_amount
            robot.add_charge(charge_amount)

        assert charge_sum <= available, "Too much charge distributed."

        # Update robot controller battery levels
        fut = yield From(self.update_battery_levels())
        raise Return(fut)

    def create_robot_manager(self, robot_name, tree, robot, position, t, battery_level, parents):
        """
        Overriding with robot manager with more capabilities.
        :param robot_name:
        :param tree:
        :param robot:
        :param position:
        :param t:
        :param battery_level:
        :param parents:
        :return:
        """
        return Robot(self.conf, robot_name, tree, robot, position, t, battery_level, parents)

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
            f.writerow([n, t, robot.robot.id,
                        robot.age(), ds.norm(), robot.velocity(),
                        robot.displacement_velocity(), robot.fitness(),
                        robot.get_battery_level(), robot.size])

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

            self.births = 0
            self.deaths = 0

            # Generate a starting population
            trees, bboxes = yield From(self.generate_population(conf.initial_population_size))
            insert_queue = zip(trees, bboxes, [None for _ in range(len(trees))])

        # Simple loop timing mechanism
        timers = {}
        this = self

        def timer(name, t):
            """
            :param t:
            :param name:
            :return:
            """
            if name not in timers:
                timers[name] = Time()

            if this.last_time is not None and float(this.last_time - timers[name]) > t:
                timers[name] = this.last_time
                return True

            return False

        # Some variables
        real_time = time.time()
        rtf_interval = 10.0
        sleep_time = 0.1
        run_result = 'unknown'
        started = False

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

            if timer('snapshot', 100.0):
                # Snapshot the world every 100 simulation seconds
                yield From(self.create_snapshot())
                yield From(wait_for(self.pause(False)))

            if timer('death', 3.0):
                # Kill off robots over their age every 5 simulation seconds
                futs = yield From(self.kill_old_robots())
                if futs:
                    yield From(multi_future(futs))

            if timer('reproduce', 3.0):
                # Attempt a reproduction every 3 simulation seconds
                potential_parents = self.select_parents()
                if potential_parents:
                    ra = random.choice(potential_parents)
                    rb = self.select_optimal_mate(ra)
                    result = yield From(self.attempt_mate(ra, rb))

                    if result:
                        child, bbox = result
                        insert_queue.append((child, bbox, (ra, rb)))

            if timer('charge', 2.0):
                # Update robot battery levels every 2 sim seconds
                # (this sends a bunch of messages, so less often saves time)
                yield From(wait_for(self.charge_batteries()))

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

            # Stop conditions
            num_bots = len(self.robots)
            age = float(self.age())

            if num_bots <= conf.extinction_cutoff:
                print("%d or fewer robots left in population - extinction." %
                      conf.extinction_cutoff)
                run_result = 'extinction'
                break
            elif num_bots >= conf.explosion_cutoff:
                print("%d or more robots in population - explosion." % conf.explosion_cutoff)
                run_result = 'explosion'
                break
            elif age > conf.stability_cutoff:
                print("World older than %f seconds, stable." % conf.stability_cutoff)
                run_result = 'stable'
                break

            yield From(trollius.sleep(sleep_time))

        # Delete all robots and reset the world, just in case a new run
        # will be started.
        self.write_result(run_result)
        yield From(wait_for(self.delete_all_robots()))
        yield From(wait_for(self.reset()))
        yield From(wait_for(self.pause(True)))
        yield From(trollius.sleep(0.5))

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
