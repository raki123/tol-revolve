from __future__ import print_function
import os
import sys
import random
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

import trollius
from trollius import From

from tol.manage import World
from revolve.convert.yaml import yaml_to_robot
from tol.spec import body_spec, brain_spec
from tol.config import Config
from tol.build import get_builder, get_simulation_robot

bot_yaml = '''
---
body:
  id          : Core
  type        : Core
  children:
    0:
        id: Light
        type: LightSensor
    1:
        id: Touch
        type: TouchSensor
    2:
        id: Brick
        type: FixedBrick
        children:
            1:
                id: ParametricBarJoint
                type: ParametricBarJoint
    3:
        id: AHinge
        type: ActiveHinge
'''

bot_yaml = '''
---
body:
  id          : Core
  type        : Core
  children:
    0:
        id: Light
        type: LightSensor
    1:
        id: Touch
        type: TouchSensor
'''


@trollius.coroutine
def run():
    conf = Config(analyzer_address=None)
    bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
    builder = get_builder(conf)
    counter = 0
    world = yield From(World.create(conf))
    yield From(world.pause())

    for _ in range(2):
        name = "test_bot_%d" % counter
        sdf = get_simulation_robot(bot, name, builder, conf)
        fut = yield From(world.insert_model(sdf))
        yield From(fut)
        print("Inserted #%d" % counter)
        yield From(world.pause(False))
        yield From(trollius.sleep(0.1))

        fut = yield From(world.delete_model(name, "delete_robot"))
        yield From(fut)
        print("Deleted #%d" % counter)
        yield From(trollius.sleep(0.1))
        yield From(world.pause(True))
        counter += 1


if __name__ == '__main__':
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")

