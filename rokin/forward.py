import numpy as np


def get_frames(q, robot):
    return robot.get_frames(q)


def get_frames_jac(q, robot):
    return robot.get_frames_jac(q=q)


def get_frames_x(q, robot):
    return robot.get_frames(q=q)[..., :-1, -1]


def frames2pos(f, f_idx, x_rel):
    return (f[..., f_idx, :, :] @ x_rel[..., np.newaxis])[..., :-1, 0]


def frames2pos_spheres(f, robot):
    """
    x_spheres f.shape[:-2] + (n_spheres, n_dim)
    """
    return frames2pos(f=f, f_idx=robot.spheres_f_idx, x_rel=robot.spheres_pos)


def frames2spheres_jac(f, j, robot):
    """
    x_spheres (n_samples, n_wp, n_spheres, n_dim)
    dx_dq (n_samples, n_wp, n_dof, n_spheres, n_dim)
    """
    x = frames2pos_spheres(f=f, robot=robot)
    # dx_dq = (j[..., robot.spheres_f_idx, :, :,] @ robot.spheres_pos[:, :, np.newaxis])[..., :-1, 0]
    dx_dq = (j[..., robot.spheres_f_idx, :, :, :] * robot.spheres_pos[:, np.newaxis, :,  np.newaxis]
             ).sum(axis=-2)[..., :-1, :]
    return x, dx_dq


def get_x_spheres(q, robot,
                  return_frames2=False):
    f = robot.get_frames(q=q)
    x_spheres = frames2pos_spheres(f=f, robot=robot)
    if return_frames2:
        return f, x_spheres
    else:
        return x_spheres


def get_x_spheres_jac(*, q, robot,
                      return_frames2=False):
    f, df_dq = robot.get_frames_jac(q=q)
    x, dx_dq = frames2spheres_jac(f=f, j=df_dq, robot=robot)
    if return_frames2:
        return (f, df_dq), (x, dx_dq)
    else:
        return x, dx_dq


# Helper
def create_frames_dict(f, robot):
    *shape, n_frames, n_dim, n_dim = f.shape
    d = np.zeros(shape + [n_frames, n_frames, n_dim, n_dim])

    for i in range(n_frames-1, -1, -1):
        nf_i = robot.next_frame_idx[i]
        ff_i = np.nonzero(robot.frame_frame_influence[i])[0][1:]

        d[..., i, i, :, :] = f[..., i, :, :]

        if isinstance(nf_i, (list, tuple)):
            nf_i = [nf_i2 for nf_i2 in nf_i for _ in range(robot.frame_frame_influence[nf_i2].sum())]
        elif nf_i == -1:
            continue

        d[..., i, ff_i, :, :] = f[..., i:i+1, :, :] @ d[..., nf_i, ff_i, :, :]

    return d


def combine_frames(f, prev_frame_idx):
    for i, pfi in enumerate(prev_frame_idx[1:], start=1):
        f[..., i, :, :] = f[..., pfi, :, :] @ f[..., i, :, :]


def combine_frames_jac(j, d, robot):
    pfi_ = robot.prev_frame_idx[robot.joint_frame_idx]
    joints_ = np.arange(robot.n_dof)[pfi_ != -1]
    jf_idx_first_ = robot.joint_frame_idx[pfi_ != -1]
    pfi_ = pfi_[pfi_ != -1]

    # Previous to joint
    # j[..., jf_idx_first_, :, :, joints_] = (d[..., [0], pfi_, :, :] @ j[..., jf_idx_first_, :, :, joints_])
    j[..., jf_idx_first_, :, :, joints_] = (np.moveaxis(d[..., [0], pfi_, :, :], -3, 0)
                                            @ j[..., jf_idx_first_, :, :, joints_])
    # After joint
    for i in range(robot.n_dof):
        nfi_i = robot.next_frame_idx[robot.joint_frame_idx[i]]
        jf_inf_i = robot.joint_frame_influence[i, :]
        jf_inf_i[:robot.joint_frame_idx[i] + 1] = False

        jf_idx_i = robot.joint_frame_idx[i]

        if isinstance(nfi_i, (list, tuple)):
            jf_inf_i = np.nonzero(robot.frame_frame_influence[i])[0]
            nfi_i = [nf_i2 for nf_i2 in nfi_i for _ in range(robot.frame_frame_influence[nf_i2].sum())]

        elif nfi_i == -1:
            continue

        temp = (j[..., jf_idx_i:jf_idx_i + 1, :, :, i] @ d[..., nfi_i, jf_inf_i, :, :])
        j[..., jf_inf_i, :, :, i] = np.moveaxis(temp, -3, 0)
        # https://stackoverflow.com/questions/67461803/resulting-shape-for-advanced-indexing-in-numpy


def get_torques(f,
                torque_f_idx, frame_frame_influence,
                mass, mass_pos, mass_f_idx,
                gravity=None,
                mode='f'):
    # Finding the torque about a given axis does not depend on the specific location on the axis where the torque acts

    *shape, n_frames, _, _ = f.shape

    if gravity is None:
        gravity = np.array([[0., 0., -9.81]])  # matrix / s**2
    force = mass[np.newaxis, :, np.newaxis] * gravity[:, np.newaxis, :]
    x_mass = frames2pos(f=f, f_idx=mass_f_idx, x_rel=mass_pos)

    torques_around_point = np.empty(shape + [len(torque_f_idx), 3])
    for i, idx_frame in enumerate(torque_f_idx):
        x_frame = f[..., idx_frame, :3, -1]
        mass_bool = frame_frame_influence[idx_frame][mass_f_idx]
        r = x_mass[..., mass_bool, :] - x_frame[..., np.newaxis, :]
        torques_around_point[..., i, :] = np.cross(r, force[:, mass_bool, :]).sum(axis=-2)

    if mode == 'dh':
        # IDENTIFICATION OF GEOMETRIC AND NON GEOMETRIC PARAMETERS OF ROBOTS, J.L. Caenen, J.C. Ange, 1990
        # Torque_x = (0_M_j - 0_P_(i-1)) x (m_j*g) @ x_(i-1)
        # Torque_y = (0_M_j - 0_P_(i-1)) x (m_j*g) @ y_(i-1)
        # Torque_z = (0_M_j - 0_P_i)     x (m_j*g) @ z_i
        f_rot_xy = np.swapaxes(f[..., torque_f_idx - 1, :3, 0:2], -1, -2)
        f_rot_z = np.swapaxes(f[..., torque_f_idx, :3, 2:3], -1, -2)
        f_rot = np.concatenate((f_rot_xy, f_rot_z), axis=-2)

    elif mode == 'f':
        f_rot = np.swapaxes(f[..., torque_f_idx, :3, :3], -1, -2)
    else:
        raise ValueError(f"Unknown mode {mode}")
    torques_around_axes = (f_rot @ torques_around_point[..., np.newaxis])[..., 0]

    return torques_around_point, torques_around_axes
