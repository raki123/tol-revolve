from __future__ import absolute_import
import math
from revolve.generate import FixedOrientationBodyGenerator
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
    "ParametricBarJoint": PartSpec(
        body_part=ParametricBarJoint,
        arity=2,
        params=[ParamSpec(
            "connection_length",
            default=50,
            min_value=20,
            max_value=100
        ), ParamSpec(
            "alpha",
            default=0,
            min_value=-0.5*math.pi,
            max_value=0.5*math.pi
        ), ParamSpec(
            "beta",
            default=0,
            min_value=0,
            max_value=math.pi
        )] + color_params
    ),
    "Wheel": PartSpec(
        body_part=Wheel,
        arity=1,
        params=color_params + [ParamSpec("radius", min_value=40, max_value=80, default=60)]
    ),
    "ActiveWheel": PartSpec(
        body_part=ActiveWheel,
        arity=1,
        outputs=1,
        params=color_params + [ParamSpec("radius", min_value=40, max_value=80, default=60)]
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
body_gen = FixedOrientationBodyGenerator(
    body_spec,

    # Only "Core" can serve as a root node
    root_parts=["Core"],

    # All other parts can potentially be attached
    attach_parts=["ActiveHinge", "Hinge", "FixedBrick",
                  "LightSensor", "TouchSensor", "ParametricBarJoint"],

    # High number of maximum parts, limit will probably be something else
    max_parts=15,

    # Maximum number of sensory inputs
    max_inputs=8,

    # Maximum number of motor outputs
    max_outputs=12
)
