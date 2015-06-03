from __future__ import absolute_import
from revolve.generate import BodyGenerator
from revolve.spec import BodyImplementation, PartSpec, ParamSpec
from ..body_parts import *

# A utility function to generate color property parameters
channel_func = lambda channel: ParamSpec(channel, min_value=0, max_value=1, default=0.5)
color_params = [channel_func("red"), channel_func("green"), channel_func("blue")]

# Body specification
body_spec = BodyImplementation({
    "Core": PartSpec(
        body_part=CoreComponent,
        arity=6,
        inputs=6,
        params=color_params
    ),
    "FixedBrick": PartSpec(
        body_part=FixedBrick,
        arity=6,
        params=color_params
    ),
    "ActiveHinge": PartSpec(
        body_part=ActiveHinge,
        arity=2,
        outputs=1,
        params=color_params
    ),
    "Hinge": PartSpec(
        body_part=Hinge,
        arity=2,
        params=color_params
    ),
    "LightSensor": PartSpec(
        body_part=LightSensor,
        arity=1,
        inputs=1,
        params=color_params
    ),
    "TouchSensor": PartSpec(
        body_part=TouchSensor,
        arity=1,
        inputs=2,
        params=color_params
    )
})

# Body generator
body_gen = BodyGenerator(
    body_spec,

    # Only "Core" can serve as a root node
    root_parts=["Core"],

    # All other parts can potentially be attached
    attach_parts=["ActiveHinge", "Hinge", "FixedBrick",
                  "LightSensor", "TouchSensor"],

    # High number of maximum parts, limit will probably be something else
    max_parts=15,

    # Maximum number of sensory inputs
    max_inputs=8,

    # Maximum number of motor outputs
    max_outputs=12
)
