import sys


ancestors = {}
sums = len(sys.argv) > 1 and sys.argv[1] == "--sums"

if sums:
    print("exp,run,id,count")
else:
    print("exp,run,id,ancestor")


def process_directory(d, exp):
    with open('/media/expdata/'+d+'/robots.csv', 'rb') as f:
        f.readline()
        for line in f:
            parts = line.split(",")
            run = parts[0]
            robot_id = parts[1]

            p1, p2 = parts[3], parts[4]
            if p1:
                parents = {p1, p2}
                parents.update(ancestors[p1], ancestors[p2])
            else:
                parents = set()

            ancestors[robot_id] = parents

            if sums:
                print("%s,%s,%s,%d" % (exp, run, robot_id, len(parents)))
            else:
                for ancestor in parents:
                    print("%s,%s,%s,%s" % (exp, run, robot_id, ancestor))

process_directory('output/plus', 'plus')
process_directory('output/plus-gradual', 'plus-gradual')
process_directory('output/plus-gradual-more', 'plus-gradual-more')
process_directory('online-output/d0-5', 'embodied')
process_directory('online-output/d6-21', 'embodied')
process_directory('online-output/d23-31', 'embodied')