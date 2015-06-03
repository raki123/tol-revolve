from revolve.spec import Robot

from tol.spec.body import body_gen, body_spec
from tol.spec.brain import brain_gen


def generate_robot(robot_id=0):
    """
    Generates a random ToL Protobuf robot
    :param robot_id:
    :return:
    :rtype: Robot
    """
    # Create a protobuf robot
    robot = Robot()
    robot.id = robot_id

    # Generate a body
    body = body_gen.generate()
    robot.body.CopyFrom(body)

    # Generate a brain
    brain = brain_gen.generate_from_body(body, body_spec)
    robot.brain.CopyFrom(brain)

    return robot
