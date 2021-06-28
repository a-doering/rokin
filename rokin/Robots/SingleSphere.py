import numpy as np
from rokin.Kinematic.Robots import Robot


class SingleSphere(Robot):
    def __init__(self, n_dof, radius=0.3):
        self.id = f'SingleSphere0{n_dof}'
        self.n_dof = self.n_dim = n_dof

        self.f_world_robot = None
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.limits = np.zeros((self.n_dof, 2))
        self.limits[:, 1] = 10

        self.spheres_pos = np.zeros((1, self.n_dim + 1))
        self.spheres_pos[:, -1] = 1
        self.spheres_rad = np.full(1, fill_value=radius)
        self.spheres_f_idx = np.zeros(1, dtype=int)

        self.n_frames = 1
        # self.next_frame_idx = np.array([-1])
        # self.prev_frame_idx = np.array([-1])
        # self.joint_frame_idx = np.zeros((0,))
        # self.joint_frame_influence = np.ones((0, 1))

    def get_frames(self, q):
        f = self._init_f(shape=q.shape, mode='eye')
        f[..., :-1, -1] += q[..., np.newaxis, :]
        return f

    def get_frames_jac(self, q):
        f = self.get_frames(q)
        j = self._init_j(shape=q.shape)
        _fill_frames_jac__dx(j=j, n_dim=self.n_dim)
        return f, j


class SingleSphere02(SingleSphere):
    def __init__(self, radius):
        super().__init__(n_dof=2, radius=radius)


class SingleSphere03(SingleSphere):
    def __init__(self, radius):
        super().__init__(n_dof=3, radius=radius)


def _fill_frames_jac__dx(j, n_dim):
    """
    Assume that the dof xy(z) are the first 2(3)
    """
    for i in range(n_dim):
        j[..., :, i, -1, i] = 1
