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
