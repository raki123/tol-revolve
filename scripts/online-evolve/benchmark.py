from online_evolve import OnlineEvoManager, parser
from trollius import From
import trollius
from revolve.util import wait_for
import time
import csv
import os


@trollius.coroutine
def run():
    """
    :return:
    """
    conf = parser.parse_args()
    conf.world_step_size = 0.004

    fname = conf.output_directory+"/benchmark.csv"
    exists = os.path.exists(fname)
    if exists:
        f = open(fname, 'ab', buffering=1)
    else:
        f = open(fname, 'wb', buffering=1)

    output = csv.writer(f, delimiter=',')

    if not exists:
        output.writerow(['run', 'population_size', 'step_size',
                         'sim_time', 'real_time', 'factor'])

    world = yield From(OnlineEvoManager.create(conf))
    yield From(wait_for(world.pause(False)))

    population_sizes = [5, 10, 15, 20, 25, 30]
    sim_time = 5
    runs = 20

    for n in population_sizes:
        for i in range(runs):
            trees, bboxes = yield From(world.generate_population(n))
            for tree, bbox in zip(trees, bboxes):
                res = yield From(world.birth(tree, bbox, None))
                yield From(res)
                yield From(trollius.sleep(0.05))

            while world.last_time is None:
                yield From(trollius.sleep(0.1))

            sim_before = world.last_time
            before = time.time()

            while float(world.last_time - sim_before) < sim_time:
                yield From(trollius.sleep(0.1))

            sim_diff = float(world.last_time - sim_before)
            diff = time.time() - before

            output.writerow((i, n, conf.world_step_size, sim_diff,
                             diff, sim_diff / diff))

            yield From(wait_for(world.delete_all_robots()))
            yield From(trollius.sleep(0.3))

    f.close()


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()