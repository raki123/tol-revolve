from sdfbuilder import SDF, Model, Link, Element
from sdfbuilder.physics import Friction
from sdfbuilder.structure import Mesh, Visual, Collision
from sdfbuilder.math import Vector3
from sdfbuilder.util import number_format as nf

from ..config import constants

# Radius and height without scaling
MESH_DIAMETER = 4.0
MESH_HEIGHT = 4.4


class BirthClinic(Model):
    """
    Birth clinic model, consists of two cylinders, one of which is rotated.
    """

    def __init__(self, diameter=3.0, name="birth_clinic"):
        """

        :param diameter: Intended diameter of the birth clinic
        :param name:
        :return:
        """
        super(BirthClinic, self).__init__(name=name, static=True)

        scale = diameter / MESH_DIAMETER
        self.diameter = diameter
        self.height = scale * MESH_HEIGHT

        mesh = Mesh("model://tol_robot/meshes/BirthClinic.dae", scale=scale)

        col = Collision("bc_col", mesh)
        surf = Element(tag_name="surface")
        friction = Friction(
            friction=0.1,
            friction2=0.1,
            slip1=0.9,
            slip2=0.9
        )
        contact = "<contact>" \
                  "<ode>" \
                  "<kd>%s</kd>" \
                  "<kp>%s</kp>" \
                  "</ode>" \
                  "</contact>" % (
                      nf(constants.SURFACE_KD), nf(constants.SURFACE_KP)
                  )
        surf.add_element(friction)
        surf.add_element(contact)
        col.add_element(surf)

        vis = Visual("bc_vis", mesh.copy())
        self.link = Link("bc_link", elements=[col, vis])

        # Translate up such that clinic is on the ground for convenience
        self.link.translate(Vector3(0, 0, 0.5 * self.height))
        self.add_element(self.link)


if __name__ == '__main__':
    sdf = SDF(elements=[BirthClinic()])
    print(str(sdf))
