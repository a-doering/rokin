from rokin.Kinematic.Robots import MovingArm03
from mopla.Justin import parameter_mobile_base as jbp


class JustinBase03(MovingArm03):
    def __init__(self):
        super().__init__()
        self.id = 'JustinBase03'
        self.spheres_pos = jbp.SPHERES_POS.copy()
        self.spheres_rad = jbp.SPHERES_RAD.copy()
        self.spheres_f_idx = jbp.SPHERES_F_IDX.copy()
