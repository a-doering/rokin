import numpy as np

from wzk import safe_scalar2array, apply_eye_wrapper
from wzk.spatial.transform_2d import fill_frames_2d_sc

from rokin import chain, forward, Robots
from rokin.SelfCollision.self_collision import get_collision_matrix_frames


def get_world_limits(n_dof, limb_lengths):
    world_limits = np.empty((2, 2))
    world_limits[:, 0] = -(n_dof + 0.5) * limb_lengths
    world_limits[:, 1] = +(n_dof + 0.5) * limb_lengths
    return world_limits


def get_arm2d_spheres(n_links, spheres_per_link, limb_lengths):
    """radius * 2 * np.cos(np.arcsin(1 / 2))"""
    n_dim = 2
    n_dim1 = n_dim+1

    spheres_per_link, limb_lengths = safe_scalar2array(spheres_per_link, limb_lengths, shape=n_links)

    n_spheres = np.sum(spheres_per_link) + 1
    spheres_pos = np.zeros((n_spheres, n_dim1))
    spheres_pos[:, -1] = 1
    spheres_pos[1:, 0] = np.concatenate([np.linspace(0, ll, spl+1)[1:] for ll, spl
                                         in zip(limb_lengths, spheres_per_link)])

    spheres_f_idx = np.zeros(n_spheres, dtype=int)
    spheres_f_idx[1:] = np.concatenate([np.full(spl, i) for i, spl in enumerate(spheres_per_link)])

    return spheres_pos, spheres_f_idx


class StaticArm(Robots.Robot):
    def __init__(self, n_dof, limb_lengths=0.5, radius=0.1, limits=None):
        self.id = "StaticArm{:0>2}".format(n_dof)
        self.n_dim = 2
        self.n_dof = n_dof

        self.f_world_robot = None
        if limits is None:
            self.infinity_joints = np.ones(self.n_dof, dtype=bool)
            limits = [-np.pi, np.pi]
        self.limits = np.repeat(np.atleast_2d(limits), self.n_dof, axis=0)

        self.limb_lengths = limb_lengths
        self.spheres_pos, self.spheres_f_idx = \
            get_arm2d_spheres(n_links=self.n_dof, spheres_per_link=3, limb_lengths=self.limb_lengths)
        self.spheres_rad = safe_scalar2array(radius, shape=len(self.spheres_f_idx))
        _init_serial_kinematic(robot=self, n_dof=self.n_dof)
        self.n_frames = len(self.next_frame_idx)

        exclude_ranges, include_pairs, exclude_pairs = get_self_collision_pairs(n_dof)
        self.self_collision_matrix = get_collision_matrix_frames(n=self.n_frames,
                                                                 exclude_ranges=exclude_ranges,
                                                                 include_pairs=include_pairs,
                                                                 exclude_pairs=exclude_pairs)

    def get_frames(self, q):
        f = self._init_f(shape=q.shape, mode='hm')
        sin, cos = np.sin(q), np.cos(q)
        _fill_frames(f=f, sin=sin, cos=cos, limb_lengths=self.limb_lengths)
        forward.combine_frames(f=f, prev_frame_idx=self.prev_frame_idx)
        f = apply_eye_wrapper(f=f, possible_eye=self.f_world_robot)
        return f

    def get_frames_jac(self, q):
        f, j = self._init_f(shape=q.shape, mode='hm'), self._init_j(shape=q.shape)

        sin, cos = np.sin(q), np.cos(q)
        _fill_frames_jac(sin=sin, cos=cos, f=f, j=j,
                         joint_frame_idx=self.joint_frame_idx, limb_lengths=self.limb_lengths)

        dh_dict = forward.create_frames_dict(f=f, robot=self)
        forward.combine_frames_jac(j=j, d=dh_dict, robot=self)
        f = dh_dict[..., 0, :, :, :]
        f = apply_eye_wrapper(f=f, possible_eye=self.f_world_robot)
        return f, j


class StaticTree(Robots.Robot):

    # TODO make this easier usable, make it as close to par as possible
    def __init__(self):
        self.next_frame_idx = [1, [2, 4], 3, -1, 5, -1]
        self.prev_frame_idx = chain.next2prev_frame_idx(nfi=self.next_frame_idx)
        self.joint_frame_idx = np.arange(len(self.next_frame_idx))
        self.joint_frame_influence = chain.influence_frames_frames2joints_frames(jfi=self.joint_frame_idx,
                                                                                 nfi=self.next_frame_idx)

        self.n_dim = 2
        self.n_dof = len(self.next_frame_idx)

        self.f_world_robot = None

        self.limb_lengths = 0.5
        self.infinity_joints = np.ones(self.n_dof, dtype=bool)
        self.limits = np.repeat(np.array([[-np.pi, np.pi]]), self.n_dof, axis=0)

        self.spheres_pos, self.spheres_f_idx = \
            get_arm2d_spheres(n_links=self.n_dof, spheres_per_link=3, limb_lengths=self.limb_lengths)

        self.spheres_rad = safe_scalar2array(0.1, shape=len(self.spheres_f_idx))

    def get_frames_jac(self, q):
        raise NotImplementedError

    def get_frames(self, q):
        raise NotImplementedError


def _fill_frames(f,
                 sin, cos,
                 limb_lengths):
    fill_frames_2d_sc(sin=sin, cos=cos, f=f[..., :-1, :, :])
    f[..., 1:, 0, -1] = limb_lengths
    f[..., -1, :-1, :-1] = np.eye(2)


def _fill_jac(j,
              sin, cos,
              joint_frame_idx):
    n_dof = j.shape[-1]
    # TODO if joint_frame_idx is not a slice, the view wont work, restructure for the more general case
    #   not efficient in this way, --> restructure
    for i in range(n_dof):
        fill_frames_2d_sc(f=j[..., joint_frame_idx[i], :, :, i], sin=cos[..., i], cos=-sin[..., i])


def _fill_frames_jac(f, j,
                     sin, cos,
                     joint_frame_idx, limb_lengths):
    _fill_frames(sin=sin, cos=cos, f=f, limb_lengths=limb_lengths)
    _fill_jac(sin=sin, cos=cos, j=j, joint_frame_idx=joint_frame_idx)


def _init_serial_kinematic(robot, n_dof):
    """
    Include a TCP after the last frame
    """
    robot.next_frame_idx = np.arange(1, n_dof + 2)
    robot.next_frame_idx[-1] = -1
    robot.joint_frame_idx = np.arange(n_dof)

    chain.complete_chain_parameters(robot=robot)


# Self Collision
def get_self_collision_pairs(n_dof):

    exclude_ranges = []
    include_pairs = []
    exclude_pairs = [(i, i+1) for i in range(0, n_dof-1)]  # no collision between neighboring links
    return exclude_ranges, include_pairs, exclude_pairs
