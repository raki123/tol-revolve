from __future__ import print_function

import random
import sys
import os
import csv
from start import OnlineEvolutionSupervisor, manager_cmd, analyzer_cmd, \
    world_file, parser

# Parameter options to try, from left to right they should lead to
# more lenient parameters (i.e. bigger chance for populations to survive)
AVG = 7.5
BASE = AVG * 3600
param_options = {
    '--max-robot-charge-rate': [a * AVG for a in (0.5, 1.0, 1.2)],
    '--max-robot-charge': [AVG * 36000],
    '--initial-charge': [BASE * a for a in (0.5, 1)],
    '--charger-r': [0.5, 0.75, 1.0],
    '--total-charge-rate': [AVG * a for a in (5, 10)],
    '--gestation-period': [36000.0 / a for a in 10, 20, 50, 100],
    '--initial-population-size': [10, 15, 20],
    '--mating-fitness-threshold': [0.6, 0.4],
    '--max-pair-children': [1, 2],
    '--part-limit': [int(a * AVG) for a in (40, 50, 60)]
}

# Just to have a fixed manner of iteration
keys = sorted([k for k in param_options])

# Current key to browse through
idx = 0

# Maximum number of experiments to run
max_experiments = 50

# Get arguments to find the output directory
args = parser.parse_args()

# Parameter lists that have been tried
tries = set()

# Generate currently tried parameter list based on output directory
output_dir = os.path.abspath(args.output_directory)
for xpdir in sorted(os.listdir(output_dir)):
    settings = os.path.join(output_dir, xpdir, "settings.conf")
    if os.path.exists(settings):
        s_args = parser.parse_args(["@"+settings])

        gtry = []
        for opt in keys:
            opt_key = opt.replace('--', '').replace('-', '_')
            gtry.append(getattr(s_args, opt_key))

        tries.add(tuple(gtry))

# Randomly initialize the starting parameter set to something
# that hasn't been tried before.
param_set = None
while True:
    positions = {a: random.choice(range(len(param_options[a]))) for a in param_options}
    param_set = tuple(param_options[p][positions[p]] for p in keys)

    if param_set not in tries:
        break

for i in range(max_experiments):
    # Launch a supervisor with the current parameter set
    tries.add(param_set)

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

    if not os.path.exists(results_file):
        print("Experiment finished without result - usually a quit, shutting down.")
        break

    result = 0
    with open(results_file, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Skip header
        next(reader)
        for row in reader:
            if row[1] == 'explosion':
                result -= 1
            elif row[1] == 'extinction' or row[1] == 'no_births':
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
                  " until intervention (this may change the outcome).", file=sys.stderr)
            break

        # Boost another parameter
        idx = (idx + 1) % len(keys)

        # Check if we can boost the "active" key in this direction
        cur = keys[idx]
        old_pos = positions[cur]
        new_pos = positions[cur] + boost

        if 0 <= new_pos < len(param_options[cur]):
            # Go again with this parameter
            print("Changing parameter `%s` from %f to %f." % (
                cur, param_options[cur][positions[cur]], param_options[cur][new_pos]
            ))
            positions[cur] = new_pos
            param_set = tuple(param_options[p][positions[p]] for p in keys)
            if param_set in tries:
                print("Duplicate parameter set, restoring.")
                positions[cur] = old_pos
            else:
                break

        # Register another attempt
        attempts += 1

    print("Experiment %d completed." % i)
