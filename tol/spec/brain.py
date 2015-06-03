from __future__ import absolute_import
from revolve.generate import NeuralNetworkGenerator
from revolve.spec import default_neural_net
from .. import constants

# Brain specification - we use Revolve's default neural net
brain_spec = default_neural_net()

# Brain generator
brain_gen = NeuralNetworkGenerator(
    brain_spec,
    max_hidden=constants.MAX_HIDDEN_NEURONS
)
