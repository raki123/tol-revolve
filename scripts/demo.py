"""
This file is what actively manages the Gazebo world.
"""
from __future__ import print_function, absolute_import

# Add "tol" directory to Python path
from sdfbuilder.math import Vector3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return

# Revolve imports
from revolve.gazebo import connect, RequestHandler, analysis_coroutine

# ToL imports
from tol.spec import generate_robot
from tol.spec.msgs import InsertSdfRequest
from tol.config import Config
from tol.build import get_builder, get_sdf_robot

# Here, we create a world which periodically checks the world state
# and acts on it. In this demo version we do the following:
# - At startup, we check the number of robots in the world. If
#   this number is below
@trollius.coroutine
def run_server():
    # Get Configuration element
    conf = Config()

    # Make a robot builder containing this config
    builder = get_builder(conf)

    # Map of known robots in the world
    robots = {}

    # Sequencer of the next robot ID to be handed out
    robot_id = 0

    # Initialize the managers for the normal Gazebo connection
    # and the body analyzer.
    manager = yield From(connect())
    analyzer = yield From(connect(("127.0.0.1", 11346)))

    # Publisher that sends the generated robot SDF to the world plugin
    robot_creator = yield From(
        manager.advertise('/gazebo/default/insert_robot_sdf/request',
                          'tol.msgs.InsertSdfRequest')
    )

    # -- Here: create some initial robot population
    initial_robots = 1
    for i in xrange(initial_robots):
        # Generate a valid robot
        print("Generating valid robot...")
        robot = yield From(generate_valid_robot(
            robot_id + i,
            analyzer,
            builder,
            conf
        ))

        print("Robot generated. Inserting into world...")
        # TODO Handle errors
        # Tell the world to insert this model
        msg = InsertSdfRequest()
        msg.robot_id = "gen__"+str(+robot_id + i)
        sdf = get_sdf_robot(robot, msg.robot_id, builder, conf, controller_plugin=None)

        # Translate robot up so it isn't lodged in the ground
        sdf.elements[0].set_position(Vector3(0, 0, 0.5))
        msg.sdf_contents = str(sdf)
        yield From(robot_creator.publish(msg))

        print("Insertion message sent.")

    robot_id += initial_robots

    # Message ID counter
    counter = 0

    request_handler = RequestHandler(manager)
    while True:
        yield From(trollius.sleep(1.0))

        # -- Here: List robots, check if any are close, and have them "reproduce" if so
        # For improved speed we would probably have a plugin check this normally, and
        # communicate only when it is needed
        entity_list = yield From(process_entity_list(request_handler, counter))
        print(entity_list)
        counter += 1

        # -- Here: Check for robot age, and kill of the old ones

@trollius.coroutine
def generate_valid_robot(robot_id, analyzer, builder, conf, max_attempts=100):
    for i in xrange(max_attempts):
        robot = generate_robot(robot_id)
        sdf = get_sdf_robot(robot, "analyze__"+str(robot_id + i),
                            builder=builder, conf=conf)

        # TODO Handle errors
        coll, _ = yield From(analysis_coroutine(sdf, "analyze__"+str(robot_id), analyzer))

        if not coll:
            raise Return(robot)

    raise Return(None)

def process_entity_list(request_handler, counter):
    id_mask = 123123123
    msg_id = "entity_list_%d"

    # Request the message
    yield From(request_handler.do_request(id_mask + counter, data="entity_list"))

    # Wait for the response
    while not request_handler.get_response(msg_id):
        yield From(trollius.sleep(0.05))

    response = request_handler.get_response(msg_id)
    request_handler.handled(msg_id)
    raise Return(response)

def main():
    loop = trollius.get_event_loop()
    loop.run_until_complete(run_server())

if __name__ == '__main__':
    main()
