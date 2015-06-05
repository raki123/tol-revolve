import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

from revolve.analyze.sdf import analyze_body
from tol.spec import generate_robot
from tol.config import Config
from tol.build import get_builder, get_sdf_robot

conf = Config(visualize_sensors=True)
builder = get_builder(conf)

for i in range(100):
    bot = generate_robot()
    sdf = get_sdf_robot(bot, "test_bot", builder, conf)

    intersections, _ = analyze_body(sdf)
    if not intersections:
        print(str(sdf))
        break
