from __future__ import print_function
import os
import sys
import random
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from tol.spec.robot import get_tree_generator
from tol.config import Config
from tol.build import get_builder, get_simulation_robot

seed = int(sys.argv[1]) if len(sys.argv) > 1 else random.randint(0, 10000)
random.seed(seed)
print("Seed: %d" % seed, file=sys.stderr)


conf = Config()
generator = get_tree_generator(conf)
bot = generator.generate_tree().to_robot()
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "my_bot", builder, conf)
print(sdf)
