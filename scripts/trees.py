"""
Maxes:

plus 141660 23
plus-gradual 59613 10
embodied 114438 18
"""
import matplotlib.pyplot as plt
import networkx as nx

ancestors = {}
global_ranks = {}


def process_directory(d, exp):
    if exp not in ancestors:
        ancestors[exp] = {}
        global_ranks[exp] = {}

    with open('/media/expdata/'+d+'/robots.csv', 'rb') as f:
        f.readline()
        for line in f:
            parts = line.split(",")
            robot_id = parts[1]

            p1, p2 = parts[3], parts[4]
            if p1:
                parents = (p1, p2)
                rank = max(global_ranks[exp][p1], global_ranks[exp][p2]) + 1
            else:
                parents = tuple()
                rank = 0

            ancestors[exp][robot_id] = parents
            global_ranks[exp][robot_id] = rank

print("Building ancestry tree...")

process_directory('output/plus', 'plus')
process_directory('output/plus-gradual', 'plus-gradual')
process_directory('output/plus-gradual-more', 'plus-gradual-more')
process_directory('online-output/d0-5', 'embodied')
process_directory('online-output/d6-21', 'embodied')
process_directory('online-output/d23-31', 'embodied')


def collect_tree(base, robot_id, nodes, edges, positions, taken, x=0, y=0):
    if robot_id in nodes:
        return

    nodes.add(robot_id)
    parents = base[robot_id]

    while (x, y) in taken:
        x -= 1

    positions[robot_id] = (x, y)
    taken.add((x, y))

    if parents:
        p1, p2 = parents
        edges.add((p1, robot_id))
        edges.add((p2, robot_id))

        collect_tree(base, p1, nodes, edges, positions, taken, x-1, y+1)
        collect_tree(base, p2, nodes, edges, positions, taken, x+1, y+1)


def create_graph(exp, robot_id):
    base = ancestors[exp]
    nodes = set()
    edges = set()
    positions = {}
    taken = set()
    collect_tree(base, robot_id, nodes, edges, positions, taken)

    row_ranges = {}
    min_x = max_x = 0
    max_y = 0
    for r in positions:
        x, y = positions[r]
        if y not in row_ranges:
            row_ranges[y] = x, x
        else:
            curmin, curmax = row_ranges[y]
            row_ranges[y] = min(x, curmin), max(x, curmax)

        min_x = min(x, min_x)
        max_x = max(x, max_x)
        max_y = max(y, max_y)

    full_center = int(0.5 * (max_x - min_x))
    row_centers = {}

    for y in row_ranges:
        minx, maxx = row_ranges[y]
        row_centers[y] = int(0.5 * (maxx + minx))

    pos = {}
    for r in positions:
        x, y = positions[r]
        offset = full_center - row_centers[y]
        x += offset
        pos[r] = x, y

    # TODO Place images on nodes:
    # https://www.wakari.io/sharing/bundle/nvikram/Basics%20of%20Networkx?has_login=False
    y_size = 70
    x_size = int(0.45 * y_size * (max_x - min_x) / max_y)
    plt.figure(figsize=(x_size, y_size))
    plt.axis('off')

    graph = nx.DiGraph()
    graph.add_edges_from(edges)

    nx.draw_networkx_nodes(graph, pos, nodelist=list(nodes), node_size=11000, node_shape='s')
    nx.draw_networkx_edges(graph, pos, edgelist=list(edges), arrows=True)
    labels = {a: a for a in nodes}
    nx.draw_networkx_labels(graph, pos, labels)

    plt.savefig('/home/elte/mt/Thesis/robot_images/trees/%s_%s_tree.png' % (exp, robot_id))
    plt.show()

    return nodes

print("Creating graphs...")
create_graph('plus', '141660')
create_graph('plus-gradual', '59613')
create_graph('embodied', '114438')
