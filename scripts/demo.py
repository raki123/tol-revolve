"""
This file is what actively manages the Gazebo world.
"""
from __future__ import print_function, absolute_import

# Add "tol" directory to Python path
import random
import itertools
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return

# Revolve imports
from revolve.gazebo import connect, RequestHandler, analysis_coroutine, get_analysis_robot
from revolve.build.util import in_mm

# sdfbuilder imports
from sdfbuilder.math import Vector3

# Uncomment these three lines to make the robots bigger - note that this
# will mess up forces so it won't lead to a good simulation.
# from revolve.build import util
# util.size_scale_factor = 10
# util.weight_scale_factor = util.size_scale_factor**3

# ToL imports
from tol.spec import get_tree_generator
from tol.spec.msgs import InsertSdfRequest
from tol.config import Config
from tol.build import get_builder, get_simulation_robot


msg_base = random.randint(0, 9999999)
if len(sys.argv) > 1:
    seed = int(sys.argv[1])
else:
    seed = random.randint(0, 1000000)

# A good stability-testing bot seems to come up with seed 126839
random.seed(seed)
print("Seed: %d" % seed)

# Some variables
MATING_DISTANCE = in_mm(200)

# Here, we create a world which periodically checks the world state
# and acts on it. In this demo version we do the following:
# - At startup, we check the number of robots in the world. If
#   this number is below
@trollius.coroutine
def run_server():
    # Get Configuration element
    conf = Config(
        update_rate=25
    )

    # Make a robot builder containing this config
    builder = get_builder(conf)

    # Instantiate a tree generator
    generator = get_tree_generator(conf)

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
    grid_size = (3, 3)
    yield From(generate_population(robot_id, grid_size, robot_creator, analyzer, generator, builder, conf))
    robot_id += grid_size[0] * grid_size[1]

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

        # -- Here: Check for robot age, and kill off the old ones

@trollius.coroutine
def generate_population(id_start, grid_size, creator, analyzer, generator, builder, conf):
    for i, j in itertools.product(range(grid_size[0]), range(grid_size[1])):
        # Generate a valid robot
        print("Generating valid robot...")
        current_id = id_start + i * grid_size[1] + j
        tree, robot, bbox = yield From(generate_valid_robot(
            current_id,
            analyzer,
            generator,
            builder
        ))

        print("Robot generated. Inserting into world...")
        # TODO Handle errors
        # Tell the world to insert this model
        msg = InsertSdfRequest()
        msg.robot_id = "gen__"+str(current_id)
        sdf = get_simulation_robot(robot, msg.robot_id, builder, conf)

        # with open('outfile.sdf', 'w') as f:
        #     f.write(str(sdf))

        # Determine x, y, z position - we translate upwards so the robot is (most likely)
        # not lodged in the ground. The bounding box is unfortunately still inaccurate.
        x = 3 * MATING_DISTANCE * i
        y = 3 * MATING_DISTANCE * j
        pos = Vector3(x, y, 0.5 * bbox[2] + 0.1)
        sdf.elements[0].set_position(pos)
        msg.sdf_contents = str(sdf)

        yield From(creator.publish(msg))
        yield From(trollius.sleep(0.05))
        print("Insertion message sent. Robot inserted at: %s" % str(pos))

@trollius.coroutine
def generate_valid_robot(robot_id, analyzer, generator, builder, max_attempts=100):
    for i in xrange(max_attempts):
        tree = generator.generate_tree()
        robot = tree.to_robot(robot_id)
        sdf = get_analysis_robot(robot, builder=builder)

        # TODO Handle errors
        coll, bbox = yield From(analysis_coroutine(sdf, analyzer))

        if not coll:
            raise Return(tree, robot, bbox)

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
