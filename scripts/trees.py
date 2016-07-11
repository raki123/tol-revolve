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


def collect_tree(base, robot_id, parents, nodes, edges, positions, taken, x=0, y=0):
    if robot_id in nodes:
        cur_pos = positions[robot_id]

        # Aim for the lowest possible position in the tree
        if cur_pos[1] > y:
            del positions[robot_id]
            taken.remove(cur_pos)
        else:
            return

    nodes.add(robot_id)

    while (x, y) in taken:
        x -= 1

    positions[robot_id] = (x, y)
    taken.add((x, y))

    node_parents = base[robot_id]
    if node_parents:
        p1, p2 = node_parents
        edges.add((p1, robot_id))
        edges.add((p2, robot_id))

        collect_tree(base, p1, parents, nodes, edges, positions, taken, x-1, y+1)
        collect_tree(base, p2, parents, nodes, edges, positions, taken, x+1, y+1)
    else:
        parents.add(robot_id)


def create_graph(exp, robot_id, levels=4, pic_factor=2.75):
    base = ancestors[exp]
    nodes = set()
    edges = set()
    positions = {}
    taken = set()
    parents = set()
    collect_tree(base, robot_id, parents, nodes, edges, positions, taken)

    row_numbers = {}

    for r in positions:
        x, y = positions[r]

        # Skip the middle part of the plot
        if y > levels:
            continue

        if y not in row_numbers:
            row_numbers[y] = [r]
        else:
            row_numbers[y].append(r)

    spread = max(len(row_numbers[y]) for y in row_numbers)
    bound = 0.5 * spread

    pos = {}
    for r in positions:
        x, y = positions[r]

        if y not in row_numbers:
            continue

        n = len(row_numbers[y])
        if n == 1:
            x = 0
        else:
            sp = np.linspace(-bound, bound, n)
            idx = row_numbers[y].index(r)
            x = float(sp[idx])

        pos[r] = x, y

    y_size = 50
    x_size = int(0.8 * y_size * spread / (levels + 1))
    fig = plt.figure(figsize=(x_size, y_size))
    ax = plt.subplot()
    plt.axis('off')

    # Parents and parent positions
    parents = list(parents)
    parent_x = np.linspace(-bound, bound, len(parents)) if len(parents) > 1 else [0]
    parent_positions = {parents[i]: (parent_x[i], levels + 0.5) for i in range(len(parents))}

    graph = nx.DiGraph()
    nodes = pos.keys()
    edges = [edge for edge in edges if edge[0] in nodes and edge[1] in nodes]
    graph.add_nodes_from(nodes)
    graph.add_nodes_from(parents)
    graph.add_edges_from(edges)

    nx.draw_networkx_nodes(graph, parent_positions, nodelist=parents, node_shape='s', node_size=15000)
    nx.draw_networkx_nodes(graph, pos, nodelist=nodes, node_shape='s', node_size=15000)
    nx.draw_networkx_edges(graph, pos, edgelist=edges, arrows=True)

    # Print dashed line
    plt.plot([-bound - 0.4, bound + 0.4], [levels + 0.25, levels + 0.25],
             linestyle='dashed', color='red', lw=10)

    # Place images on nodes:
    # https://www.wakari.io/sharing/bundle/nvikram/Basics%20of%20Networkx?has_login=False
    trans = ax.transData.transform
    trans2 = fig.transFigure.inverted().transform

    for each_node in graph:
        img_path = "/home/elte/mt/Thesis/robot_images/morph/%s_%s.png" % (exp, each_node)
        if not os.path.exists(img_path):
            print("%s %s" % (exp, each_node))
            continue

        xx, yy = trans(pos[each_node] if each_node in pos else parent_positions[each_node])
        xa, ya = trans2((xx, yy))

        with open(img_path, 'rb') as f:
            img = Image.open(f)
            w, h = img.size

            # this is the image size
            factor = y_size * pic_factor
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
# create_graph('plus', '141660', 4, 2.75)
create_graph('plus-gradual', '59613', 4, 7.0)
# create_graph('embodied', '114438')
