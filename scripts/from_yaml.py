from __future__ import print_function
from sdfbuilder.math import Vector3
from sdfbuilder.structure import Collision
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

# from revolve.build import util
# util.size_scale_factor = 10

from revolve.convert.yaml import yaml_to_robot
from tol.spec import get_body_spec, get_brain_spec
from tol.config import parser
from tol.build import get_builder, get_simulation_robot

# bot_yaml = '''
# ---
# body:
#   id          : Core
#   type        : Core
#   children:
#     0:
#       id: Brick1
#       type: FixedBrick
#       orientation: 90
#       children:
#         2:
#           id: Sensor
#           type: TouchSensor
#     1:
#       id: Hinge
#       type: Hinge
#       orientation: 90
#     2:
#       id: Tc1
#       type: TouchSensor
#
# '''
bot_yaml = '''
---
body:
  id          : Core
  type        : Core
  children:
    0:
      id: T1
      type: ParametricBarJoint
      children:
        1:
          id: C3
          type: FixedBrick
    1:
      id: T2
      type: ActiveHinge
      children:
        1:
          id: C1
          type: FixedBrick
    2:
      id: T3
      type: Hinge
      children:
        1:
          id: C2
          type: FixedBrick
    3:
      id: T4
      type: FixedBrick
'''
conf = parser.parse_args()
body_spec = get_body_spec(conf)
brain_spec = get_brain_spec(conf)
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
sdf.elements[0].set_position(Vector3(0, 0, 0.05))

with open("/home/elte/.gazebo/models/test_bot/model.sdf", "w") as f:
    f.write(str(sdf))

