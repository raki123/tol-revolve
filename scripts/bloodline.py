
print("exp,run,id,level")


def process_directory(d, exp):
    levels = {}

    with open('/media/expdata/'+d+'/robots.csv', 'rb') as f:
        f.readline()
        for line in f:
            parts = line.split(",")
            run = parts[0]
            robot_id = parts[1]

            p1, p2 = parts[3], parts[4]

            if p1:
                level = max(levels[p1], levels[p2]) + 1
            else:
                level = 1

            print("%s,%s,%s,%s" % (exp, run, robot_id, level))
            levels[robot_id] = level


process_directory('output/plus', 'plus')
process_directory('output/plus-gradual', 'plus-gradual')
process_directory('output/plus-gradual-more', 'plus-gradual-more')
process_directory('online-output/d0-5', 'embodied')
process_directory('online-output/d6-21', 'embodied')
process_directory('online-output/d23-31', 'embodied')
