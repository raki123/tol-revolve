from __future__ import print_function
from sdfbuilder.math import Vector3
from sdfbuilder.structure import Collision
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# from revolve.build import util
# util.size_scale_factor = 10

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
      id: PassiveHinge
      type: Hinge
      children:
        1:
          id: Touch
          type: TouchSensor
    1:
      id: ActiveHinge
      type: ActiveHinge
      params:
        red: 1
        green: 0
        blue: 0
      children:
        1:
          id: Brick
          type: FixedBrick
brain:
  params:
    ActiveHinge-out-0:
      type: Oscillator
      period: 8
      amplitude: 2
'''

conf = Config(visualize_sensors=True)
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
sdf.elements[0].set_position(Vector3(0, 0, 0.02325))
print(str(sdf))
