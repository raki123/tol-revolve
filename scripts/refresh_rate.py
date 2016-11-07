import sys

lookahead = int(sys.argv[1])

collect = {}
with open("/media/expdata/output/alive.csv", "rb") as f:
    f.readline()

    for line in f:
        exp, run, births, robot_id = line.split(',')
        births = int(births)

        if exp not in collect:
            collect[exp] = {}

        if run not in collect[exp]:
            collect[exp][run] = {}

        if births not in collect[exp][run]:
            collect[exp][run][births] = set()

        collect[exp][run][births].add(robot_id)

print("exp,run,births,total,still_alive")
for exp in collect:
    exp_data = collect[exp]

    for run in exp_data:
        run_data = exp_data[run]
        max_births = max(run_data.keys())

        for births in run_data:
            next_idx = int(births) + lookahead

            if next_idx > max_births:
                break

            if next_idx not in run_data:
                raise RuntimeError("Cannot calculate refresh rate for %d, data "
                                   "not available." % lookahead)

            a, b = run_data[births], run_data[next_idx]
            diff = a.intersection(b)
            print("%s,%s,%s,%d,%d" % (exp, run, births, len(a), len(diff)))