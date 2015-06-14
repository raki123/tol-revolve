"""
This file is what actively manages the Gazebo world.
"""
from __future__ import print_function, absolute_import

# Add "tol" directory to Python path
import random
from sdfbuilder.math import Vector3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return

# Revolve imports
from revolve.gazebo import connect, RequestHandler, analysis_coroutine, get_analysis_robot

# Uncomment these three lines to make the robots bigger - note that this
# will mess up forces so it won't lead to a good simulation.
# from revolve.build import util
# util.size_scale_factor = 10
# util.weight_scale_factor = util.size_scale_factor**3

# ToL imports
from tol.spec import generate_robot
from tol.spec.msgs import InsertSdfRequest
from tol.config import Config
from tol.build import get_builder, get_simulation_robot


msg_base = random.randint(0, 9999999)
if len(sys.argv) > 1:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 1000000)

random.seed(seed)
print("Seed: %d" % seed)

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
    robot_id = msg_base

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
    # We'll toss the bots in a 5m x 5m square at random positions
    initial_robots = 1
    yield From(generate_population(robot_id, initial_robots, 2.5, 2.5,
                                   robot_creator, analyzer, builder, conf))
    robot_id += initial_robots

    # Message ID counter
    counter = 0

    request_handler = RequestHandler(manager)
    while False:
        yield From(trollius.sleep(1.0))

        # -- Here: List robots, check if any are close, and have them "reproduce" if so
        # For improved speed we would probably have a plugin check this normally, and
        # communicate only when it is needed
        entity_list = yield From(process_entity_list(request_handler, counter))
        print(entity_list)
        counter += 1

        # -- Here: Check for robot age, and kill of the old ones

def generate_population(id_start, n_bots, max_x, max_y, creator, analyzer, builder, conf):
    for i in xrange(n_bots):
        # Generate a valid robot
        print("Generating valid robot...")
        robot, bbox = yield From(generate_valid_robot(
            id_start + i,
            analyzer,
            builder
        ))

        print("Robot generated. Inserting into world...")
        # TODO Handle errors
        # Tell the world to insert this model
        msg = InsertSdfRequest()
        msg.robot_id = "gen__"+str(id_start + i)
        sdf = get_simulation_robot(robot, msg.robot_id, builder, conf)

        with open('outfile.sdf', 'w') as f:
            f.write(str(sdf))

        # Determine x, y, z position - we translate upwards so the robot is (most likely)
        # not lodged in the ground. The bounding box is unfortunately still inaccurate.
        x = random.uniform(0, max_x)
        y = random.uniform(0, max_y)
        pos = Vector3(0, 0, 0.1)
        sdf.elements[0].set_position(pos)
        msg.sdf_contents = str(sdf)

        yield From(creator.publish(msg))
        yield From(trollius.sleep(0.05))
        print("Insertion message sent. Robot inserted at: %s" % str(pos))

@trollius.coroutine
def generate_valid_robot(robot_id, analyzer, builder, max_attempts=100):
    for i in xrange(max_attempts):
        robot = generate_robot(robot_id)
        sdf = get_analysis_robot(robot, builder=builder)

        # TODO Handle errors
        coll, bbox = yield From(analysis_coroutine(sdf, analyzer))

        if not coll:
            raise Return(robot, bbox)

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
