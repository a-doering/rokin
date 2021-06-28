import numpy as np
from sympy import Matrix, cos, sin, pi as sy_pi, symbols, simplify


__dh_mode_fix = 'fix'
__dh_mode_var = 'var'

name_joint = 'q'
name_dh5 = ['dh_d', 'dh_theta', 'dh_a', 'dh_alpha', 'dh_beta']


def pi_ify(x, f=(0, 1, 1/2, 1/3, 2/3, 1/4, 3/4, 1/5, 2/5, 3/5, 4/5)):
    x2 = np.copy(x).astype(object)
    for ff in f:
        x2[np.isclose(x, np.pi*ff)] = sy_pi * ff
        x2[np.isclose(x, -np.pi*ff)] = -sy_pi * ff
    return x2


def simplify2(frames, inverse=True, rational=False, ratio=10):
    for i, f in enumerate(frames.flat):
        if f is not None:
            frames.flat[i] = simplify(f, inverse=inverse, rational=rational, ratio=ratio)


def evaluate(frame, v_list, values):
    return frame.evalf(subs={v_list[i]: values[i] for i in range(len(v_list))})


def rot_x(alpha):
    return Matrix([[1, 0, 0, 0],
                   [0, +cos(alpha), -sin(alpha), 0],
                   [0, +sin(alpha), +cos(alpha), 0],
                   [0, 0, 0, 1]])


def rot_y(beta):
    return Matrix([[+cos(beta), 0, +sin(beta), 0],
                   [0, 1, 0, 0],
                   [-sin(beta), 0, +cos(beta), 0],
                   [0, 0, 0, 1]])


def rot_z(gamma):  # theta
    return Matrix([[+cos(gamma), -sin(gamma), 0, 0],
                   [+sin(gamma), +cos(gamma), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def trans_x(a):
    return Matrix([[1, 0, 0, a],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def trans_y(b):
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, b],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def trans_z(c):  # d
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, c],
                   [0, 0, 0, 1]])


def frames_from_dh(*, q, d, theta, a, alpha, beta=None):
    """
    beta !=0  (rotation around y) is only used if two consecutive frames are parallel (parallel z axis == alpha=0)
    """
    if beta is None:
        return rot_x(alpha) * trans_x(a) * rot_z(theta+q) * trans_z(d)
    else:
        return rot_y(beta) * rot_x(alpha) * trans_x(a) * rot_z(theta+q) * trans_z(d)
        # return rot_x(alpha) * trans_x(a) * rot_z(theta+q) * trans_z(d) * rot_y(beta)


def frames_from_dh2(q, dh):
    if dh.shape[1] == 4:
        return [frames_from_dh(q=q_i, d=d_i, theta=theta_i, a=a_i, alpha=alpha_i)
                for q_i, (d_i, theta_i, a_i, alpha_i) in zip(q, dh)]
    elif dh.shape[1] == 5:
        return [frames_from_dh(q=q_i, d=d_i, theta=theta_i, a=a_i, alpha=alpha_i, beta=beta_i)
                for q_i, (d_i, theta_i, a_i, alpha_i, beta_i) in zip(q, dh)]
    else:
        raise AttributeError('There are only 4 DH parameters; 5 for parallel axis')


def create_q_dh(n_dof, passive_joints):
    """naming passive or coupled"""
    q0 = symbols(f"{name_joint}{0}:{n_dof}")
    if passive_joints:

        keys = np.array(list(passive_joints.keys()))
        n = n_dof + len(keys)
        q = tuple(q0[i - (keys < i).sum()] if i not in passive_joints
                  else passive_joints[i](q0)
                  for i in range(n))
        return q0, q

    else:
        return q0, q0


def create_dh(dh, mode):
    if mode == 'fix':
        dh = pi_ify(x=dh)
    elif mode == 'var4':
        n_dh = len(dh)
        dh = np.stack([symbols(f"{x}{0}:{n_dh}") for x in name_dh5[:4]]).T
    elif mode == 'var5':
        n_dh = len(dh)
        dh = np.stack([symbols(f"{x}{0}:{n_dh}") for x in name_dh5]).T
    else:
        raise ValueError

    return dh


def get_frames(q, dh, robot):

    frames = np.zeros(robot.n_frames, dtype=object)
    frames[robot.joint_frame_idx_dh] = frames_from_dh2(q=q, dh=dh)

    if robot.f_idx_static is not None and robot.f_static is not None:
        frames[robot.f_idx_static] = [Matrix(pi_ify(f)) for f in robot.f_static]

    return frames


def get_jacs(frames, q0):
    n_frames = len(frames)
    n_dof = len(q0)

    jacs = np.zeros((n_dof, n_frames), dtype=object)

    for i in range(n_dof):
        for j in range(n_frames):
            jac_ij = frames[j].diff(q0[i])
            if len(np.nonzero(jac_ij)[0]) != 0:
                jacs[i, j] = jac_ij
            else:
                jacs[i, j] = None

    return jacs


def get_frames_jacs(robot, dh_mode):

    q0, q = create_q_dh(n_dof=robot.n_dof, passive_joints=robot.coupled_passive_joints)
    dh = create_dh(dh=robot.dh, mode=dh_mode)

    frames = get_frames(q=q, dh=dh, robot=robot)

    simplify2(frames)
    frames_jac = get_jacs(frames=frames, q0=q0)
    simplify2(frames_jac)

    return frames, frames_jac
