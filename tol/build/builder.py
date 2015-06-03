from revolve.build.sdf import RobotBuilder, BodyBuilder, NeuralNetBuilder
from sdfbuilder import SDF
from ..spec import body_spec, brain_spec
from ..config import Config


def get_builder(conf):
    """
    :param conf:
    :return:
    """
    return RobotBuilder(BodyBuilder(body_spec, conf), NeuralNetBuilder(brain_spec))


def get_sdf_robot(robot, name, builder, conf, controller_plugin=None):
    """
    :param robot:
    :param name:
    :param builder:
    :param conf: Config
    :type conf: Config
    :param controller_plugin:
    :return:
    """
    model = builder.get_sdf_model(robot, controller_plugin=controller_plugin,
                                  update_rate=conf.update_rate, name=name)
    sdf = SDF()
    sdf.add_element(model)
    return sdf

