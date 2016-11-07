import uuid
from revolve.spec.msgs import Robot
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from tol.spec import get_body_spec
from tol.config import parser, make_revolve_config
from sdfbuilder.math import Vector3

conf = make_revolve_config(parser.parse_args())
body_spec = get_body_spec(conf)

print("exp,run,robot_id,gene_id,origin_id,type,r,g,b")


def find_part_uuid(type, vec, parent_parts):
    for pid, ptype, pvec, porigin in parent_parts:
        if ptype == type and abs(vec - pvec) < 1e-8:
            return pid, porigin

    return None, None


def process_parts(robot_id, part, parent_parts, cur=None):
    # Last three parameters are always color
    type, vec = part.type, Vector3(*[p.value for p in part.param[-3:]])
    pid, porigin = find_part_uuid(type, vec, parent_parts)

    if pid is None:
        pid, porigin = uuid.uuid4(), robot_id

    entry = (pid, type, vec, porigin)

    if cur is None:
        cur = [entry]
    else:
        cur.append(entry)

    for conn in part.child:
        process_parts(robot_id, conn.part, parent_parts, cur)

    return cur


def track_genes(basedir, robot_id, p1, p2, gene_map):
    bot = Robot()
    with open(basedir+'/robot_'+robot_id+'.pb', 'rb') as f:
        bot.ParseFromString(f.read())

    if p1:
        parent_parts = gene_map[p1] + gene_map[p2]
    else:
        parent_parts = []

    entries = process_parts(robot_id, bot.body.root, parent_parts)
    gene_map[robot_id] = entries

    return entries


def process_directory(d, exp):
    gene_map = {}
    basedir = '/media/expdata/'+d
    with open(basedir+'/robots.csv', 'rb') as f:
        f.readline()

        for line in f:
            parts = line.split(",")
            run = parts[0]
            robot_id = parts[1]

            p1, p2 = parts[3], parts[4]

            entries = track_genes(basedir, robot_id, p1, p2, gene_map)
            for pid, type, vec, origin in entries:
                print('%s,%s,%s,%s,%s,%s,%f,%f,%f' % (exp, run, robot_id, pid, origin,
                                                      type, vec.x, vec.y, vec.z))

process_directory('output/baseline-random', 'baseline-random')
process_directory('output/baseline-selection', 'baseline-selection')
process_directory('output/plus', 'plus')
process_directory('output/plus-gradual', 'plus-gradual')
process_directory('output/plus-gradual-more', 'plus-gradual-more')
process_directory('online-output/d0-5', 'embodied')
process_directory('online-output/d6-21', 'embodied')
process_directory('online-output/d23-31', 'embodied')
