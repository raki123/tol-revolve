"""
Includes functions for robot tree analysis, to:

- Determine the number of extremities per robot
- Determine the number of joints per robot
-
"""
import itertools
from revolve.angle.representation import Tree, Node


def count_extremities(node):
    """
    Counts the extremities in the subtree represented by the given node.
    :type node: Node
    """
    # Count all leaf nodes to get the extremities
    return len([c for c in node.get_children()
                if not len(list(c.child_connections()))])


def count_joints(node):
    """
    Counts the number of joints in the subtree represented by the given node.
    :type node: Node
    """
    types = ("ActiveHinge", "Hinge")
    base = 1 if node.part.type in types else 0
    return base + len([c for c in node.get_children() if c.part.type in types])


def joints_per_extremity(node):
    """
    Returns a list with the number of joints encountered in each extremity
    in the subtree represented by the given node.
    :type node: Node
    """
    # If the number of extremities is 1, the recursion is
    # broken here and we return the number of joints on
    # this subtree
    if count_extremities(node) == 1:
        return [count_joints(node)]

    # Recursively find the start of each extremity and return the joints / extremity
    # count for it.
    return itertools.chain([joints_per_extremity(c.node) for c in node.child_connections()])
