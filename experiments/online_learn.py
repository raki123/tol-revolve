# Online learn scheme
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
import time
import os
import shutil
import random
import csv
import itertools
import logging
import trollius
from trollius import From, Return

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')

from sdfbuilder import Pose
from sdfbuilder.math import Vector3

from revolve.util import wait_for

from tol.manage.robot import Robot
from tol.config import parser
from tol.manage import World
from tol.logging import logger, output_console
from tol.util.analyze import list_extremities, count_joints, count_motors, count_extremities, count_connections

# Log output to console
output_console()
logger.setLevel(logging.DEBUG)


def str2bool(v):
    return v.lower() == "true" or v == "1"


class OnlineLearnManager(World):
    """
    Extended world manager for the online learn script
    """

    def __init__(self, conf, _private):
        """

        :param conf:
        :param _private:
        :return:
        """
        super(OnlineLearnManager, self).__init__(conf, _private)

        self._snapshot_data = {}

        # Output files
        csvs = {
            'generations': ['run', 'gen', 'robot_id', 'vel', 'dvel', 'fitness', 't_eval'],
            'robot_details': ['robot_id', 'extremity_id', 'extremity_size', 'joint_count', 'motor_count']
        }
        self.csv_files = {
            k: {
                'filename': None,
                'file': None,
                'csv': None,
                'header': csvs[k]
            } for k in csvs
            }

        self.current_run = 0

        if self.output_directory:
            for k in self.csv_files:
                fname = os.path.join(self.output_directory, k + '.csv')
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

    def robots_header(self):
        return Robot.header()

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
    def create_snapshot(self):
        """
        Copy the generations file in the snapshot
        :return:
        """
        ret = yield From(super(OnlineLearnManager, self).create_snapshot())
        if not ret:
            raise Return(ret)

        for k in self.csv_files:
            entry = self.csv_files[k]
            if entry['file']:
                entry['file'].flush()
                shutil.copy(entry['filename'], entry['filename'] + '.snapshot')

    @trollius.coroutine
    def get_snapshot_data(self):
        """
        :return:
        """
        data = yield From(super(OnlineLearnManager, self).get_snapshot_data())
        data.update(self._snapshot_data)
        raise Return(data)

    @trollius.coroutine
    def evaluate_pair(self, tree, bbox, parents=None):
        """
        Evaluates a single robot tree.
        :param tree:
        :param bbox:
        :param parents:
        :return: Evaluated Robot object
        """
        # Pause the world just in case it wasn't already
        yield From(wait_for(self.pause(True)))

        pose = Pose(position=Vector3(0, 0, -bbox.min.z))
        fut = yield From(self.insert_robot(tree, pose, parents=parents))
        robot = yield From(fut)

        max_age = self.conf.evaluation_time + self.conf.warmup_time

        # Unpause the world to start evaluation
        yield From(wait_for(self.pause(False)))

        before = time.time()

        while True:
            if robot.age() >= max_age:
                break

            # Sleep for the pose update frequency, which is about when
            # we expect a new age update.
            yield From(trollius.sleep(1.0 / self.state_update_frequency))

        yield From(wait_for(self.delete_robot(robot)))
        yield From(wait_for(self.pause(True)))

        diff = time.time() - before
        if diff > self.conf.evaluation_threshold:
            sys.stderr.write("Evaluation threshold exceeded, shutting down with nonzero status code.\n")
            sys.stderr.flush()
            sys.exit(15)

        raise Return(robot)

    @trollius.coroutine
    def run(self):
        """
        :return:
        """
        conf = self.conf

        if self.do_restore:
            # Recover from a previously cancelled / crashed experiment
            data = self.do_restore
            evo_start = data['evo_start']
            gen_start = data['gen_start']
            pairs = data['local_pairs']
        else:
            # Start at the first experiment
            evo_start = 1
            gen_start = 1
            pairs = None

        for evo in range(evo_start, conf.num_evolutions + 1):
            self.current_run = evo

            if not pairs:
                # Only create initial population if we are not restoring from
                # a previous experiment.
                trees, bboxes = yield From(self.generate_population(conf.population_size))
                pairs = yield From(self.evaluate_population(trees, bboxes))
                self.log_generation(evo, 0, pairs)

            for generation in xrange(gen_start, conf.num_generations):
                if (generation % 10) == 0:
                    # Snapshot every 10 generations
                    self._snapshot_data = {
                        "local_pairs": pairs,
                        "gen_start": generation,
                        "evo_start": evo
                    }
                    yield From(self.create_snapshot())
                    print("Created snapshot of experiment state.")

                # Produce the next generation and evaluate them
                robots = [p[0] for p in pairs]
                if conf.disable_evolution:
                    child_trees, child_bboxes = yield From(
                        self.generate_population(conf.population_size))
                    parent_pairs = None
                else:
                    child_trees, child_bboxes, parent_pairs = yield From(self.produce_generation(robots))

                child_pairs = yield From(self.evaluate_population(child_trees, child_bboxes, parent_pairs))

                if conf.keep_parents:
                    pairs += child_pairs
                else:
                    pairs = child_pairs

                # Sort the bots and reduce to population size
                if conf.disable_fitness:
                    random.shuffle(pairs)
                else:
                    pairs = sorted(pairs, key=lambda r: r[0].fitness(), reverse=True)

                pairs = pairs[:conf.population_size]
                self.log_generation(evo, generation, pairs)

            # Clear "restore" parameters
            gen_start = 1
            pairs = None

        yield From(self.teardown())

    @trollius.coroutine
    def teardown(self):
        """
        :return:
        """
        yield From(super(OnlineLearnManager, self).teardown())
        for k in self.csv_files:
            if self.csv_files[k]['file']:
                self.csv_files[k]['file'].close()


@trollius.coroutine
def run():
    """
    :return:
    """
    conf = parser.parse_args()
    world = yield From(OnlineLearnManager.create(conf))
    yield From(world.run())


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()
