# Revolve imports
from revolve.build.sdf import Box
from revolve.build.util import in_grams, in_mm

# Local imports
from .util import ColorMixin

width = in_mm(46.5)


class FixedBrick(Box, ColorMixin):
    """
    Brick - same size as the core component, but
    without any sensors. We can conveniently model this
    as a box.
    """
    X = width
    Y = width
    Z = width
    MASS = in_grams(14.9)

    def _initialize(self, **kwargs):
        super(FixedBrick, self)._initialize(**kwargs)
        self.apply_color()