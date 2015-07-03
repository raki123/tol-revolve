from __future__ import absolute_import

from revolve.generate import NeuralNetworkGenerator
from revolve.spec import default_neural_net

from ..config import constants


# Brain specification - we use Revolve's default neural net
brain_spec = default_neural_net()

def get_brain_generator(conf):
    """
    Returns a brain generator.

    :param conf:
    :type conf: Config
    :return:
    """
    return NeuralNetworkGenerator(
        brain_spec,
        max_hidden=constants.MAX_HIDDEN_NEURONS
    )
