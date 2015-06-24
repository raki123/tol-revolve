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
    1:
      id: Rotator
      type: ActiveRotator
      children:
        1:
          id: ChildBrick
          type: FixedBrick
    2:
      id: ActiveCardan
      type: ActiveCardan
      children:
        1:
          id: Cardan
          type: Cardan
    3:
      id: Bar
      type: ParametricBarJoint
      params:
        alpha: 0.12
        beta: 0.25
    4:
      id: Touch
      type: TouchSensor
    5:
      id: Light
      type: LightSensor
'''

conf = Config(visualize_sensors=True)
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
print(str(sdf))
