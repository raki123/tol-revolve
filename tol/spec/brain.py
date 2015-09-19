from __future__ import absolute_import

from revolve.generate import NeuralNetworkGenerator
from revolve.spec import default_neural_net

from ..config import Config, constants


def get_brain_spec(conf):
    """
    Returns the brain specification corresponding to the
    given config.
    :param conf:
    :type conf: Config
    :return:
    """
    return default_neural_net(conf.brain_mutation_epsilon)


def get_brain_generator(conf):
    """
    Returns a brain generator.

    :param conf:
    :type conf: Config
    :return:
    """
    return NeuralNetworkGenerator(
        get_brain_spec(conf),
        max_hidden=constants.MAX_HIDDEN_NEURONS
    )
