from __future__ import print_function
import os
import sys
import random
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from tol.spec.robot import get_tree_generator
from tol.config import Config
from tol.build import get_builder, get_simulation_robot

from revolve.angle import Crossover, Mutator

seed = int(sys.argv[1]) if len(sys.argv) > 1 else random.randint(0, 10000)
random.seed(seed)
print("Seed: %d" % seed, file=sys.stderr)


conf = Config()
generator = get_tree_generator(conf)
crossover = Crossover(generator.body_gen, generator.brain_gen)
mutator = Mutator(generator.body_gen, generator.brain_gen,
                  p_delete_hidden_neuron=0.1,
                  p_remove_brain_connection=0.1,
                  p_delete_subtree=0.5,
                  p_swap_subtree=0.5,
                  p_duplicate_subtree=0.5)

bot1 = generator.generate_tree()
mutator.mutate(bot1)

# bot2 = generator.generate_tree()
# success, bot3 = crossover.crossover(bot1, bot2)
# print("Success: %s" % str(success), file=sys.stderr)

builder = get_builder(conf)
sdf = get_simulation_robot(bot1.to_robot(0), "my_bot", builder, conf)
print(sdf)
