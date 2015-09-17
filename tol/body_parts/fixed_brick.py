# Revolve / sdfbuilder imports
from revolve.build.sdf import Box
from revolve.build.util import in_grams, in_mm
from sdfbuilder.structure import Box as BoxGeom
from sdfbuilder.structure import Mesh

# Local imports
from .util import ColorMixin

WIDTH = in_mm(46.5)
MASS = in_grams(14.9)


class FixedBrick(Box, ColorMixin):
    """
    Brick - same size as the core component, but
    without any sensors. We can conveniently model this
    as a box.
    """
    X = WIDTH
    Y = WIDTH
    Z = WIDTH
    MASS = MASS

    def _initialize(self, **kwargs):
        self.component = self.create_component(
            BoxGeom(self.x, self.y, self.z, self.mass), "box",
            visual=Mesh("file://meshes/FixedBrick.dae"))
        self.apply_color()
