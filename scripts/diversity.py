from zss import simple_distance
from itertools import combinations
from revolve.spec import Robot
import sys

class Node(object):
    def __init__(self, part=None, conn=None):
        self.conn = conn
        self.part = part


def get_children(node):
    return [Node(conn.part, conn) for conn in sorted(node.part.child, key=lambda p: p.src)]


def label_distance(a, b):
    """
    Define node distance as follows:
    - A difference in type += 1.0
    - A difference in parent attachment slot += 1.0
    """
    if not isinstance(a, Node) or not isinstance(b, Node):
        return 1.0

    dist = 0
    if a.part.type != b.part.type:
        dist += 1.0

    if a.conn and b.conn and a.conn.dst != b.conn.dst:
        dist += 1.0

    return dist


def get_label(node):
    return node


def process_list(cur, bots, robot_cache, pair_cache):
    exp, run, births = cur

    if exp == 'embodied':
        r = int(run)
        if 0 <= r <= 5:
            subdir = 'd0-5'
        elif 6 <= r <= 21:
            subdir = 'd6-21'
        elif r == 22:
            return
        else:
            subdir = 'd23-31'

        dir = '/media/expdata/online-output/'+subdir
    else:
        dir = '/media/expdata/output/'+exp

    proto_bots = []
    for robot_id in bots:
        if robot_id in robot_cache:
            proto_bots.append(robot_cache[robot_id])
        else:
            bot = Robot()
            with open(dir+'/robot_'+str(robot_id)+'.pb', 'rb') as f:
                bot.ParseFromString(f.read())

            bot_pair = robot_id, Node(bot.body.root)
            proto_bots.append(bot_pair)
            robot_cache[robot_id] = bot_pair

    for (aid, a), (bid, b) in combinations(proto_bots, 2):
        k = (aid, bid)
        if k in pair_cache:
            diff = pair_cache[k]
        else:
            diff = simple_distance(a, b, get_children, get_label, label_distance)
            pair_cache[k] = diff

        print("%s,%s,%s,%s,%s,%f" % (exp, run, births, aid, bid, diff))


print("exp,run,births,a,b,diff")
# Process only this experiment
filter_exp = sys.argv[1]

with open('/media/expdata/output/alive.csv', 'rb') as f:
    f.readline()

    cur = None
    exp_run = None
    bots = []
    robot_cache = {}
    pair_cache = {}

    for line in f:
        exp, run, births, robot_id = line.split(',')

        if exp != filter_exp:
            continue

        idf = (exp, run, births)

        if idf != cur:
            if cur:
                process_list(cur, bots, robot_cache, pair_cache)
            bots = []
            cur = idf

            if int(births) % 10 == 0:
                sys.stderr.write("%s/%s/3000\n" % (run, births))

        if exp_run != (exp, run):
            # New robot cache
            exp_run = (exp, run)
            pair_cache = {}
            robot_cache = {}

        bots.append(robot_id.strip())

    # Process the last list
    process_list(cur, bots, robot_cache, pair_cache)
