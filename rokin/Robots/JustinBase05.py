import numpy as np

from rokin.Robots import MovingArm
from rokin.Robots import Justin19
from rokin.Robots.JustinBase03 import justinbase03_par as jbp


_justin19 = Justin19()


def _q2linklengths(q=None):
    if q is None:
        # from Justin.primitives_torso import justin_primitives
        # q = justin_primitives(justin='getready')
        ll = 0.08914747047562913
        return ll

    ll = _justin19.get_frames(q)[[0, -4], :2, -1]
    ll = np.linalg.norm(ll[1] - ll[0])
    return ll


class JustinBase05(MovingArm):
    def __init__(self, q0_justin=None):
        ll = _q2linklengths(q0_justin)
        super().__init__(n_dof=3, limb_lengths=np.array([0, ll, 0]))
        self.id = 'JustinBase05'

        self.infinity_joints = np.array([False, False, True, False, False], dtype=bool)

        self.limits = np.array([[0, 10],
                                [0, 10],
                                [-np.pi, np.pi],
                                _justin19.limits[0],
                                _justin19.limits[-2]])

        self.spheres_pos = jbp.SPHERES_POS.copy()
        self.spheres_rad = jbp.SPHERES_RAD.copy()
        self.spheres_f_idx = jbp.SPHERES_F_IDX.copy()
