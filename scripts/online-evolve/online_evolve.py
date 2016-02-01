"""
Online evolution experiment. Initially we try to answer the following questions:

- How do complexity and size evolve?
- How does this depend on `X` (variable to be determined)
- How can we make the system stable? Depending on # of babies,
  initial / max population size, age of death.
"""
from tol.config import parser

# Environment parameters
parser.add_argument(
    '--world-diameter',
    default=50, type=float,
    help="The diameter of the environment in meters."
)

parser.add_argument(
    '--birth-clinic-diameter',
    default=2, type=float,
    help="The diameter of the birth clinic in meters."
)

# General population parameters
parser.add_argument(
    '--initial-population-size',
    default=10, type=int,
    help="The size of the starting population."
)

parser.add_argument(
    '--max-population-size',
    default=50, type=int,
    help="The maximum size of the population."
)

parser.add_argument(
    '--max-lifetime',
    default=36000, type=float,
    help="The absolute maximum number of seconds a robot is"
         " allowed to live."
)

parser.add_argument(
    '--age-cutoff',
    default=0.15, type=float,
    help="A robot's age of death is determined by the formula "
         "`Ml * min(f, c)/c`, where `Ml` is the maximum lifetime, "
         "`f` is the fitness of the robot and `c` is this cutoff value. "
         "This results in a linear increase between zero and the maximum "
         "age."
)

# Mating parameters
parser.add_argument(
    '--mating-distance-threshold',
    default=1, type=float,
    help="The mating distance threshold in meters."
)

parser.add_argument(
    '--mating-fitness-threshold',
    default=0.5, type=float,
    help="The maximum fractional fitness difference between two robots that "
         "will allow a mate. E.g. for a fraction of 0.5, two robots will not mate"
         " if one is 50% less fit than the other."
)

parser.add_argument(
    '--gestation-period',
    default=36000 / 20.0, type=float,
    help="The minimum time a robot has to wait between matings."
)

parser.add_argument(
    '--max-pair-children',
    default=2, type=int,
    help="The maximum number of children one pair of robots is allowed to have."
)
