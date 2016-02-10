from __future__ import print_function

import random
import sys
import os
import csv
from start import OnlineEvolutionSupervisor, manager_cmd, analyzer_cmd, \
    world_file, parser

# Parameter options to try, from left to right they should lead to
# more lenient parameters (i.e. bigger chance for populations to survive)
param_options = {
    '--initial-population-size': [8, 10, 12, 14, 16],
    '--initial-age-mu': [36000.0 / a for a in (15, 10, 7.5, 5, 2.5)],
    '--initial-age-sigma': [36000.0 / a for a in (30, 20, 15, 10)],
    '--age-cutoff': [0.2, 0.15, 0.1, 0.05],
    '--mating-fitness-threshold': [0.6, 0.5, 0.4, 0.3, 0.2],
    '--gestation-period': [36000.0 / a for a in 10, 20, 50, 100, 150, 200],
    '--max-pair-children': [1, 2, 3, 4, 5, 6],
}

# Initial picks from the list
positions = {
    '--initial-population-size': 2,
    '--initial-age-mu': 1,
    '--initial-age-sigma': 1,
    '--age-cutoff': 2,
    '--mating-fitness-threshold': 2,
    '--gestation-period': 3,
    '--max-pair-children': 2,
}

# Just to have a fixed manner of iteration
keys = [k for k in param_options]

# Current key to browse through
idx = 0

# Maximum number of experiments to run
max_experiments = 50

args = parser.parse_args()

for i in range(max_experiments):
    # Launch a supervisor with the current parameter set
    manager_args = sys.argv[1:]
    for key, pos in positions.iteritems():
        ops = param_options[key]
        manager_args += [key, str(ops[pos])]

    supervisor = OnlineEvolutionSupervisor(
        manager_cmd=manager_cmd,
        analyzer_cmd=analyzer_cmd,
        world_file=world_file,
        output_directory=args.output_directory,
        manager_args=manager_args
    )

    supervisor.launch()

    # Read run results file to decide what to do next
    results_file = os.path.join(supervisor.snapshot_directory, 'results.csv')
    result = 0
    with open(results_file, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Skip header
        next(reader)
        for row in reader:
            if row[1] == 'explosion':
                result -= 1
            elif row[1] == 'extinction':
                result += 1
            elif row[1] != 'stable':
                print("WARNING: Unsupported result `%s`" % row[1],
                      file=sys.stderr)

    # If we have an extinct population we would like to boost it, if not
    # we'll try to decrease an option instead.
    if result == 0:
        print("Stability: try random boost...")
        boost = -1 if random.random() > 0.5 else 1
        print("...random boost is %s." % "positive" if boost > 0 else "negative")
    elif result > 0:
        print("Extinction: try positive boost.")
        boost = 1
    else:
        print("Explosion: try negative boost.")
        boost = -1

    attempts = 0
    while True:
        # If there's no key left to boost, just repeat
        # the experiment
        if attempts >= len(keys):
            print("No more adaptations are possible! Rerunning experiment"
                  " until intervention.", file=sys.stderr)
            break

        # Boost another parameter
        idx = (idx + 1) % len(keys)

        # Check if we can boost the "active" key in this direction
        cur = keys[idx]
        new_pos = positions[cur] + boost
        if 0 <= new_pos < len(param_options[cur]):
            # Go again with this parameter
            print("Changing parameter `%s` from %f to %f." % (
                cur, param_options[cur][positions[cur]], param_options[cur][new_pos]
            ))
            positions[cur] = new_pos
            break
        else:
            attempts += 1

    print("Experiment %d completed." % i)
