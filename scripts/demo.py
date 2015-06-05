"""
This file is what actively manages the Gazebo world.
"""
# Add "tol" directory to Python path
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# Trollius / Pygazebo
import trollius
from trollius import From, Return

# Revolve imports
from revolve.gazebo import connect, RequestHandler

# ToL imports
from tol.spec import generate_robot
from tol.config import Config
from tol.build import get_builder, get_sdf_robot

# Here, we create a world which periodically checks the world state
# and acts on it. In this demo version we do the following:
# - At startup, we check the number of robots in the world. If
#   this number is below
@trollius.coroutine
def run_server():
    # Map of known robots in the world
    robots = {}

    # Initialize the managers for the normal Gazebo connection
    # and the body analyzer.
    gz_manager = yield From(connect())
    analyzer = yield From(connect(("127.0.0.1", 11346)))

    # -- Here: create some initial robot population

    # Message ID counter
    counter = 0

    request_handler = RequestHandler(gz_manager)
    while True:
        yield From(trollius.sleep(0.1))

        # -- Here: List robots, check if any are close, and have them "reproduce" if so
        # For improved speed we would probably have a plugin check this normally, and
        # communicate only when it is needed
        entity_list = yield From(process_entity_list(request_handler, counter))
        counter += 1

        # -- Here: Check for robot age, and kill of the old ones

def process_entity_list(request_handler, counter):
    msg_id = "entity_list_%d"

    # Request the message
    yield From(request_handler.do_request(msg_id % counter, data="entity_list"))

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
