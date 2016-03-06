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

bot_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
    2:
      id          : Leg20Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg20
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg21Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg21
                  type        : FixedBrick
    3:
      id          : Leg30Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg30
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg31Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg31
                  type        : FixedBrick


brain:
  # extra input neuron (bias)
  neurons:
    Core-hidden-0:
      id: Core-hidden-0
      layer: hidden
      part_id: Core
      type: Oscillator

  # Here you specify the connections between neurons, as
  # {"src": "src-id", "dst": "dst-id", "weight": float}
  connections:
  - src: Core-hidden-0
    dst: Leg00Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg01Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg10Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg11Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg20Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg21Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg30Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg31Joint-out-0
    weight: 1.0


  params:
    Core-hidden-0:
      period: 1.0
      phase_offset: 0
      amplitude: 1.8

'''
# bot_yaml = '''
# ---
# body:
#   id          : Core
#   type        : TouchSensor
# '''
conf = parser.parse_args()
body_spec = get_body_spec(conf)
brain_spec = get_brain_spec(conf)
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_simulation_robot(bot, "test_bot", builder, conf)
sdf.elements[0].set_position(Vector3(0, 0, 0.1))

with open("/home/elte/.gazebo/models/test_bot/model.sdf", "w") as f:
    f.write(str(sdf))

