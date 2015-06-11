from __future__ import print_function
from sdfbuilder import Link
from sdfbuilder.math import Vector3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from revolve.convert.yaml import yaml_to_robot
from tol.spec import body_spec, brain_spec
from tol.config import Config
from tol.build import get_builder, get_simulation_robot

bot_yaml = '''
---
body:
  id          : Core
  type        : FixedBrick
  children:
    0:
      id:   Brick1
      type: FixedBrick
      children:
        4:
          id: Brick2
          type: FixedBrick
          children:
            3:
              id: Brick3
              type: FixedBrick
    2:
      id: Brick4
      type: FixedBrick
      children:
        1:
          id: Brick5
          type: FixedBrick
          children:
            3:
              id: Tested
              type: ParametricBarJoint
              params:
                connection_length: 70
                alpha: 0.5
                beta: 0.5
              children:
                1:
                  id: Brick6
                  type: FixedBrick
'''

conf = Config()
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
sdf.elements[0].translate(Vector3(0, 0, 0.5))
print(str(sdf))
