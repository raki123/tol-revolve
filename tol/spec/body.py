from __future__ import absolute_import
from revolve.angle.robogen.spec import BodyGenerator
from revolve.angle.robogen.spec import get_body_spec as rv_body_spec


def get_body_spec(conf):
    return rv_body_spec(conf)


def get_body_generator(conf):
    """
    Returns a body generator.

    :param conf:
    :return:
    """
    return BodyGenerator(conf)
