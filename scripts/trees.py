"""
Maxes:

plus 141660 23
plus-gradual 59613 10
embodied 114438 18
"""
import matplotlib.pyplot as plt
import networkx as nx
from PIL import Image
import os
import numpy as np

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


def create_graph(exp, robot_id, from_bottom=3, from_top=3):
    base = ancestors[exp]
    nodes = set()
    edges = set()
    positions = {}
    taken = set()
    collect_tree(base, robot_id, nodes, edges, positions, taken)

    max_y = max(positions[r][1] for r in positions)
    row_ranges = {}
    row_numbers = {}
    min_x = max_x = 0
    top_start = max_y - from_top

    for r in positions:
        x, y = positions[r]

        # Skip the middle part of the plot
        if from_bottom < y <= top_start:
            continue

        if y not in row_ranges:
            row_ranges[y] = x, x
            row_numbers[y] = []
        else:
            curmin, curmax = row_ranges[y]
            row_ranges[y] = min(x, curmin), max(x, curmax)
            row_numbers[y].append(r)

        min_x = min(x, min_x)
        max_x = max(x, max_x)

    full_center = int(0.5 * (max_x - min_x))
    row_centers = {}

    for y in row_ranges:
        minx, maxx = row_ranges[y]
        row_centers[y] = int(0.5 * (maxx + minx))

    pos = {}
    for r in positions:
        x, y = positions[r]

        if y not in row_numbers:
            continue

        offset = full_center - row_centers[y]
        x += offset

        if y > top_start:
            y -= top_start - from_bottom

        pos[r] = x, y

    y_size = 50
    x_size = int(0.45 * y_size * (max_x - min_x) / (from_bottom + from_top + 1))
    fig = plt.figure(figsize=(x_size, y_size))
    ax = plt.subplot()
    plt.axis('off')

    graph = nx.DiGraph()
    nodes = pos.keys()
    edges = [edge for edge in edges if edge[0] in nodes and edge[1] in nodes]
    graph.add_edges_from(edges)

    nx.draw_networkx_nodes(graph, pos, nodelist=nodes, node_shape='s')
    nx.draw_networkx_edges(graph, pos, edgelist=edges, arrows=True)

    # Print dashed line
    omx = min(pos[r][0] for r in pos)
    ommx = max(pos[r][0] for r in pos)
    plt.plot([omx - 0.5, ommx + 0.5], [from_bottom + 0.5, from_bottom + 0.5],
             linestyle='dashed', color='blue', lw=10)

    # labels = {a: a for a in nodes}
    # nx.draw_networkx_labels(graph, pos, labels)

    # Place images on nodes:
    # https://www.wakari.io/sharing/bundle/nvikram/Basics%20of%20Networkx?has_login=False
    trans = ax.transData.transform
    trans2 = fig.transFigure.inverted().transform

    for each_node in graph:
        img_path = "/home/elte/mt/Thesis/robot_images/morph/%s_%s.png" % (exp, each_node)
        if not os.path.exists(img_path):
            print("%s %s" % (exp, each_node))
            continue

        xx, yy = trans(pos[each_node])
        xa, ya = trans2((xx, yy))

        with open(img_path, 'rb') as f:
            img = Image.open(f)
            w, h = img.size

            # this is the image size
            factor = y_size * 2.0
            piesize_1 = (factor / float(h))
            piesize_2 = (factor / float(w))
            p2_2 = piesize_2 / 2
            p2_1 = piesize_1 / 2
            a = plt.axes([xa - p2_2, ya - p2_1, piesize_2, piesize_1])

            # insert image into the node
            graph.node[each_node]['image'] = img

            # display it
            a.imshow(graph.node[each_node]['image'])

            # turn off the axis from minor plot
            a.axis('off')

    plt.savefig('/home/elte/mt/Thesis/robot_images/trees/%s_%s_tree.png' % (exp, robot_id),
                bbox_inches="tight", pad_inches=0.0)

    return nodes

print("Creating graphs...")
create_graph('plus', '141660')
# create_graph('plus-gradual', '59613')
# create_graph('embodied', '114438')
