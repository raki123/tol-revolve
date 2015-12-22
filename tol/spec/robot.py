from revolve.angle import TreeGenerator
from ..spec.body import get_body_generator
from ..spec.brain import get_brain_generator


def get_tree_generator(conf):
    """
    :param conf:
    :return:
    :rtype: TreeGenerator
    """
    body_gen = get_body_generator(conf)
    brain_gen = get_brain_generator(conf)
    return TreeGenerator(body_gen, brain_gen)
