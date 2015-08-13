from __future__ import print_function
from sdfbuilder.math import Vector3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')

import trollius
from trollius import From
from tol.config import Config
from tol.manage import World

from sdfbuilder import SDF, Link, Model
from sdfbuilder.sensor import Sensor

conf = Config(visualize_sensors=True)

sdf = SDF()
model = Model("crash")
link = Link("my_link")
link.make_box(1.0, 1, 1, 1)
sensor = Sensor("sense", "contact")
link.add_element(sensor)
model.add_element(link)
sdf.add_element(model)
model.set_position(Vector3(0, 0, 0.5))


@trollius.coroutine
def run_server():
    world = yield From(World.create(conf))

    counter = 0
    while True:
        model.name = "test_bot_%d" % counter
        print("Inserting %s..." % model.name)
        fut = yield From(world.insert_model(sdf))
        yield From(fut)
        print("Done. Waiting...")
        yield From(trollius.sleep(1.0))
        print("Done. Removing...")
        fut = yield From(world.delete_model(model.name))
        yield From(fut)
        print("Done. Waiting...")
        yield From(trollius.sleep(1.0))
        counter += 1


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")

if __name__ == '__main__':
    main()
