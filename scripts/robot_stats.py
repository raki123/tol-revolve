import sys
import os
import glob

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from revolve.angle.representation import Tree
from revolve.spec import Robot
from tol.spec import get_body_spec
from tol.config import parser, make_revolve_config
from tol.util.analyze import list_extremities, count_joints, count_motors


conf = parser.parse_args([])
make_revolve_config(conf)
body_spec = get_body_spec(conf)

input_dirs = sys.argv[1:]

for input_dir in input_dirs:
    input_dir = os.path.abspath(input_dir)
    print("Processing %s..." % input_dir)
    files = glob.glob(os.path.join(input_dir, "*.pb"))

    with open(os.path.join(input_dir, "robot_details.csv"), 'w') as o:
        o.write("robot_id,size,extremity_id,extremity_size,joint_count,motor_count\n")
        for filename in files:
            robot = Robot()
            with open(filename, "rb") as f:
                robot.ParseFromString(f.read())

            tree = Tree.from_body_brain(robot.body, robot.brain, body_spec)

            base = os.path.splitext(os.path.basename(filename))[0]
            robot_id = int(base.replace("robot_", ""))

            counter = 0
            for extr in list_extremities(tree.root):
                num_joints = count_joints(extr)
                num_motors = count_motors(extr)
                o.write("%d,%d,%d,%d,%d,%d\n" % (robot_id, len(tree), counter, len(extr), num_joints, num_motors))
                counter += 1
