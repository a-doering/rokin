import numpy as np

from rokin import chain
from rokin.Robots import Robot, import_robot_cpp
from rokin.Robots.Justin19 import justin19_par as jtp
from rokin.SelfCollision.self_collision import get_collision_matrix_frames


world_limits = np.array([[-2, 2],
                         [-2, 2],
                         [-0.5, 3.5]])


class CenterOfMass(object):
    __slots__ = ('base_frame_idx',     # int         | Frame of the base, center of mass should not be too far away
                 'com_frame_idx',      # int         | Frame of the center of mass
                 'eps_dist_cost',
                 'dist_threshold')


class Justin19(Robot):

    def __init__(self):
        self.id = 'Justin19'
        self.n_dim = 3
        self.n_dof = jtp.N_JOINTS
        self.n_frames = jtp.N_FRAMES
        self.limits = jtp.JOINT_LIMITS2
        self.infinity_joints = np.zeros(jtp.N_JOINTS, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = jtp.DH.copy()  # TODO load from calibration
        self.dh_elastic = jtp.DH.copy()
        self.next_frame_idx = jtp.next_frame_idx.copy()
        self.joint_frame_idx = jtp.joint_frame_idx.copy()
        chain.complete_chain_parameters(robot=self)

        self.f_static = jtp.F_STATIC
        self.f_idx_static = jtp.IDX_F_STATIC
        self.joint_frame_idx_dh = jtp.joint_frame_idx_dh.copy()
        self.coupled_passive_joints = jtp.coupled_passive_joints
        self.coupled_passive_joints_jac = jtp.coupled_passive_joints_jac

        # Sphere Model (for collisions)
        self.spheres_pos = jtp.SPHERES_POS.copy()
        self.spheres_rad = jtp.SPHERES_RAD.copy()
        self.spheres_f_idx = jtp.SPHERES_F_IDX.copy()

        # Capsule Model (for collisions)
        self.capsules_pos = jtp.CAPSULES_POS.copy()
        self.capsules_rad = jtp.CAPSULES_RAD.copy()
        self.capsules_f_idx = jtp.CAPSULES_F_IDX.copy()

        exclude_ranges, include_pairs, exclude_pairs = get_self_collision_pairs()
        self.self_collision_matrix = get_collision_matrix_frames(n=self.n_frames,
                                                                 exclude_ranges=exclude_ranges,
                                                                 include_pairs=include_pairs,
                                                                 exclude_pairs=exclude_pairs)

        # Mass and Torque Model
        # MA
        self.masses_pos = jtp.MASS_POS[:, :3]
        self.masses = jtp.MASSES
        self.masses_frame_idx = jtp.MASS_F_IDX

        # Meshes
        self.meshes = 1
        self.meshes_frames = 1
        self.meshes_frames_idx = 1

        self._cpp = import_robot_cpp(robot=self, replace=False)

    def sample_q(self, shape=None):
        q = super().sample_q(shape=shape)
        return sample_q_torso_23(q=q, limits=self.limits)

    def prune_joints2limits(self, q):
        q = np.clip(q, a_min=self.limits[:, 0], a_max=self.limits[:, 1])
        return prune_joints_to_limits_torso23(q=q, limits=self.limits, verbose=0)


# Self Collision
def get_self_collision_pairs():

    # body parts (ranges of frames) within no collision can take place, exceptions in include_pairs
    exclude_ranges = [jtp.IDX_F_TORSO,
                      jtp.IDX_F_RIGHT,
                      jtp.IDX_F_LEFT,
                      jtp.IDX_F_HEAD]
    include_pairs = []
    include_pairs += [(i, j) for i in [5, 6, 7] for j in [12, 13]]     # right shoulder -- right upper arm
    include_pairs += [(i, j) for i in [14, 15, 16] for j in [21, 22]]  # left shoulder -- left upper arm

    # frame pairs between which no collision can take place
    exclude_pairs = []
    exclude_pairs += [(i, j) for i in [4] for j in [5, 6, 7]]                   # torso -- right upper arm
    exclude_pairs += [(i, j) for i in [4] for j in [14, 15, 16]]                # torso -- left upper arm
    exclude_pairs += [(i, j) for i in jtp.IDX_F_TORSO for j in jtp.IDX_F_HEAD]  # torso -- head

    return exclude_ranges, include_pairs, exclude_pairs


# Coupled joints
def sample_q_torso_23(q, limits, safety=jtp.JOINT_LIMITS_SAFETY_FACTOR):
    (t2, t3), (c_lower, c_upper), (_, _) = helper_coupled_torso23(q=q, limits=limits, safety=safety)
    q[..., t3] = np.random.uniform(low=c_lower, high=c_upper, size=c_upper.shape)
    return q


def constraints_limits_check_torso23(q, limits, safety=jtp.JOINT_LIMITS_SAFETY_FACTOR, verbose=0):
    n_samples, n_wp, n_dof = q.shape

    (_, _), (_, _), (below_lower, above_upper) = helper_coupled_torso23(q=q, limits=limits, safety=safety)

    # Check the feasibility of each sample
    outside_limits = np.logical_or(below_lower, above_upper)
    outside_limits = outside_limits.reshape(n_samples, -1)
    outside_limits = outside_limits.sum(axis=-1)
    feasible = outside_limits == 0

    if verbose > 1:
        for i in range(np.size(feasible)):
            print('Joint Limit justin_cython 23: Feasible: {}'.format(feasible[i]))

    return feasible


def helper_coupled_torso23(q, limits, safety=jtp.JOINT_LIMITS_SAFETY_FACTOR):
    """
    # joint limits for t3
    # if t2 < 0:
    #     [0-t2, 135]
    # else:
    #     [0, 135-t2]


    :param q:
    :param limits:
    :param safety: how close to the limits the coupled joint is allowed to drive in rad
    :return:
    """
    t2, t3 = jtp.IDX_J_COUPLED_TORSO_TILTING
    q = np.atleast_2d(q)
    c_lower = np.full_like(q[..., t2], fill_value=limits[t3, 0]) + safety
    c_upper = np.full_like(q[..., t2], fill_value=limits[t3, 1]) - safety
    t2_below0 = q[..., t2] < 0
    # c_lower, c_upper, t2_below0 = np.atleast_1d(c_lower, c_upper, t2_below0)

    c_lower[t2_below0] -= q[..., t2][t2_below0]
    c_upper[~t2_below0] -= q[..., t2][~t2_below0]

    below_lower = q[..., t3] < c_lower
    above_upper = q[..., t3] > c_upper
    return (t2, t3), (c_lower, c_upper), (below_lower, above_upper)


def prune_joints_to_limits_torso23(q, limits, safety=jtp.JOINT_LIMITS_SAFETY_FACTOR, verbose=0):

    (t2, t3), (c_lower, c_upper), (below_lower, above_upper) = helper_coupled_torso23(q=q, limits=limits, safety=safety)
    q[..., t3][below_lower] = c_lower[below_lower]
    q[..., t3][above_upper] = c_upper[above_upper]

    if verbose > 0:
        count = np.sum(below_lower) + np.sum(above_upper)
        print(f"Number of joint violations: {count}")

    return q


def joint_limits_ajustin_torso23(q,
                                 limits):

    t2, t3 = jtp.IDX_J_COUPLED_TORSO_TILTING
    q = np.atleast_2d(q)

    t2_below0 = q[..., t2] < 0

    const = limits[t3, 1] - q[..., t2] - q[..., t3]
    const[t2_below0] = -limits[t3, 0] - q[..., t2][t2_below0] + q[..., t3][t2_below0]
    return const


# def joint_limits_ajustin_torso23_jac(q):
#     # joint limits for t3
#     # if t2 < 0:
#     #     [-t2, 135]   -> 0 < t3 + t2
#     # else:
#     #     [0, 135-t2]  -> 0 < 135 - t2 - t3
#
#     t2 = 1
#     t3 = 2
#
#     _, n_waypoints, n_dof = q.shape
#
#     t2_above0 = q[0, :, t2] > 0
#
#     n_variables = n_dof * n_waypoints
#
#     ic = np.arange(n_waypoints).repeat(2)
#     iv = np.concatenate((np.arange(start=t2, stop=n_variables, step=n_dof)[:, np.newaxis],
#                          np.arange(start=t3, stop=n_variables, step=n_dof)[:, np.newaxis]),
#                         axis=1).flatten()
#     data = np.ones((n_waypoints, 2))
#     data[t2_above0, :] = -1
#     limit_jac = sparse_matrix((data.flatten(), (ic, iv)), shape=(n_waypoints, n_variables))
#
#     return limit_jac


# Center of Mass
def get_com():
    com = CenterOfMass()
    com.base_frame_idx = jtp.IDX_F_TORSO_BASE
    com.com_frame_idx = jtp.FRAME_UPPER_BODY_COM
    com.eps_dist_cost = 0.05
    com.dist_threshold = 0.2
    return com


def speed_test_var_fix_dh():
    robot = Justin19()
    n = 1000
    m = 100
    q = robot.sample_q(m)
    dh4 = np.random.random((m, 20, 4))
    dh5 = np.random.random((m, 20, 5))

    from wzk import tic, toc

    time_list = []
    for i in range(n):

        tic()
        _ = robot.get_frames_jac(q=q)
        time_list.append(toc(''))
        tic()
        _ = robot.get_frames_jac_dh(q=q, dh=dh4)
        time_list.append(toc(''))
        tic()
        _ = robot.get_frames_jac_dh(q=q, dh=dh5)
        time_list.append(toc(''))

    time_fix, time_var4, time_var5 = np.array(time_list).reshape(n, 3).T

    time_fix = time_fix[time_fix < np.percentile(time_fix, q=99)]
    time_var4 = time_var4[time_var4 < np.percentile(time_var4, q=99)]
    time_var5 = time_var5[time_var5 < np.percentile(time_var5, q=99)]

    from wzk import new_fig
    fig, ax = new_fig()
    ax.hist(time_fix, bins=30, alpha=0.5, label='fix dh')
    ax.hist(time_var4, bins=30, alpha=0.5, label='var dh4')
    ax.hist(time_var5, bins=30, alpha=0.5, label='var dh5')
    ax.legend()


if __name__ == '__main__':
    pass
    # robot = Justin19()
    # q = robot.sample_q()
    # from mopla.Justin.primitives_torso import justin_primitives
    #
    # q = justin_primitives(justin='getready')
    # f = robot.get_frames(q)[13]
    #
    # b = np.array([[0, 0, 1, 0],
    #               [1, 0, 0, 0]])
