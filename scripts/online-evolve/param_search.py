import sys
import os
import csv
from start import OnlineEvolutionSupervisor, manager_cmd, analyzer_cmd, \
    world_file, parser

# Parameter options to try, from left to right they should lead to
# more lenient parameters (i.e. bigger chance for populations to survive)
param_options = {
    '--initial-population-size': [10, 12, 14, 16],
    '--initial-age-mu': [3600 / 10.0, 3600 / 7.5, 3600 / 5.0, 3600 / 2.5],
    '--initial-age-sigma': [3600 / 20.0, 3600 / 15.0, 3600 / 10.0],
    '--age-cutoff': [0.2, 0.15, 0.1, 0.05],
    '--mating-fitness-threshold': [0.6, 0.5, 0.4, 0.3],
    '--gestation-period': [36000 / 20.0, 36000 / 50.0, 36000 / 100.0, 3600 / 150.0],
    '--max-pair-children': [2, 3, 4, 5, 6],
}

# Initial picks from the list
positions = {
    '--initial-population-size': 1,
    '--initial-age-mu': 1,
    '--initial-age-sigma': 1,
    '--age-cutoff': 2,
    '--mating-fitness-threshold': 2,
    '--gestation-period': 2,
    '--max-pair-children': 2,
}

# Just to have a fixed manner of iteration
keys = [k for k in param_options]

# Current key to browse through
idx = 0

# Maximum number of experiments to run
max_experiments = 25

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

    # If we have an extinct population we would like to boost it, if not
    # we'll try to decrease an option instead.
    boost = -1 if result >= 0 else 1
    attempts = 0

    while True:
        # If there's no key left to boost, just repeat
        # the experiment
        if attempts >= len(keys):
            print("No more adaptations are possible! Rerunning experiment"
                  " until intervention.")
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
