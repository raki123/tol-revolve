from __future__ import print_function
import time
import collections
from sdfbuilder import Pose
from sdfbuilder.math import Vector3
from revolve.convert.yaml import yaml_to_robot
from revolve.util import Time
from revolve.angle import Tree
import os
import sys
from pygazebo.pygazebo import DisconnectError
import trollius
from trollius import ConnectionRefusedError, ConnectionResetError, From, Return

# Add root directory to import search path
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from tol.spec import get_body_spec, get_brain_spec
from tol.config import parser
from tol.manage import World


bot_yaml = '''
---
body:
  id          : Core
  type        : Hinge
  children:
    1:
      id: Cam
      type: LightSensor
'''


@trollius.coroutine
def sleep_sim_time(world, seconds):
    start = world.last_time if world.last_time else Time()
    remain = seconds

    while remain > 0:
        yield From(trollius.sleep(0.05))
        now = world.last_time if world.last_time else Time()
        remain = seconds - float(now - start)


@trollius.coroutine
def run_server():
    conf = parser.parse_args()
    conf.enable_light_sensor = True
    conf.output_directory = None
    conf.analyzer_address = ""

    body_spec = get_body_spec(conf)
    brain_spec = get_brain_spec(conf)
    bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
    world = yield From(World.create(conf))
    yield From(world.pause(True))

    diffs = collections.deque(maxlen=20)
    mdiffs = collections.deque(maxlen=20)
    ref = -1
    mref = -1
    sim_time_sec = 2.5

    while True:
        tree = Tree.from_body_brain(bot.body, bot.brain, body_spec)
        before = time.time()
        fut = yield From(world.insert_robot(tree, Pose(position=Vector3(z=1.0))))
        robot = yield From(fut)
        yield From(world.pause(False))
        middle = time.time()
        yield From(sleep_sim_time(world, sim_time_sec))
        after = time.time()
        yield From(world.delete_robot(robot))
        yield From(world.pause(True))

        diff = after - before
        diffs.append(diff)
        mdiff = after - middle
        mdiffs.append(mdiff)

        # fac = sim_time_sec / diff
        avg = sum(diffs) / len(diffs)
        mavg = sum(mdiffs) / len(mdiffs)
        if ref < 0 and len(diffs) == 10:
            ref = avg
            mref = mavg

        ref_diff = avg - ref
        mref_diff = mavg - mref
        # avg_fac = sim_time_sec / avg

        print("%f\t%f\t%f\t%f\t%f\t%f" % (diff, avg, ref_diff, mdiff, mavg, mref_diff))


def main():
    def handler(loop, context):
        exc = context['exception']
        if isinstance(exc, DisconnectError) or isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)

        raise context['exception']

    try:
        loop = trollius.get_event_loop()
        loop.set_exception_handler(handler)
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")
    except ConnectionRefusedError:
        print("Connection refused, are the world and the analyzer loaded?")

if __name__ == '__main__':
    main()
