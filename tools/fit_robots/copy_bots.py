import sys
import os
import shutil

base = sys.argv[1]
counter = 0
with open("paths.csv", "rb") as f:
    for line in f:
        if not len(line):
            continue

        exp, robot_id = line.split()
        basename = "robot_"+str(robot_id)
        start = os.path.join(base, exp, basename)
        shutil.copy(start+".sdf", "fit"+str(counter)+".sdf")
        shutil.copy(start+".pb", "fit"+str(counter)+".pb")
        counter += 1
