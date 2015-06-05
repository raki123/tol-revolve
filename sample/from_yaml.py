import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from revolve.convert.yaml import yaml_to_robot
from tol.spec import body_spec, brain_spec
from tol.config import Config
from tol.build import get_builder, get_sdf_robot

bot_yaml = '''
---
body:
  id          : Core
  type        : Core

  # Children maps
  children:
    1:
      id          : Hip1
      type        : ParametricBarJoint
      params      :
        alpha: -0.75
        beta: 0.78
      children    :
        1:
          id:   SubBrick
          type: FixedBrick
'''

conf = Config()
bot = yaml_to_robot(body_spec, brain_spec, bot_yaml)
builder = get_builder(conf)
sdf = get_sdf_robot(bot, "test_bot", builder, conf)
print(str(sdf))
