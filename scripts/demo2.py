# Add "tol" directory to Python path
import random
import itertools
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return

# Revolve
from revolve.build.util import in_mm

# ToL
from tol.config import Config
from tol.manage import World


# Set command line seed if supplied, otherwise choose a random number
if len(sys.argv) > 1:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 1000000)

random.seed(seed)
print("Seed: %d" % seed)

# Some variables
MATING_DISTANCE = in_mm(200)


@trollius.coroutine
def run_server():
    conf = Config(
        update_rate=25
    )

    world = yield From(World.create(conf))
    grid_size = (2, 2)
    positions = [(3 * MATING_DISTANCE * i, 3 * MATING_DISTANCE * j, 0.2)
                 for i, j in itertools.product(range(grid_size[0]), range(grid_size[1]))]

    yield From(world.generate_starting_population(positions))


def main():
    loop = trollius.get_event_loop()
    loop.run_until_complete(run_server())

if __name__ == '__main__':
    main()

