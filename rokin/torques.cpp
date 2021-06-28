MASSES = array([4.651, 5.47 , 2.925, 5.759, 2.637, 2.326, 2.239, 2.204, 1.101,
                1.641, 0.675, 2.   , 2.637, 2.326, 2.239, 2.204, 1.101, 1.641,
                0.675, 2.   , 3.   ])

MASS_F_IDX = np.array([1, 2, 3, 4,                  # Torso
                       6, 7, 8, 9, 10, 11, 12,      # Right Arm
                       13,                          # Right Hand
                       15, 16, 17, 18, 19, 20, 21,  # Left Arm
                       22,                          # Left Hand
                       25])                         # Head

array([[ 1.3342422e-03,  1.5386544e-02,  7.4260405e-03,  1.0000000e+00],
       [ 2.1859133e-01, -1.3019641e-02,  6.2231342e-03,  1.0000000e+00],
       [ 1.5320139e-01, -2.2720538e-02,  1.9282567e-03,  1.0000000e+00],
       [ 5.3666155e-02,  7.4570502e-02,  6.8257189e-04,  1.0000000e+00],
       [-4.1410813e-04,  2.0290376e-02, -4.5943169e-02,  1.0000000e+00],
       [-2.9078800e-05,  1.1406515e-01,  1.6447191e-02,  1.0000000e+00],
       [-3.0257825e-05,  1.6511890e-02, -1.1329461e-01,  1.0000000e+00],
       [-3.0879429e-05,  1.1374485e-01,  1.6369065e-02,  1.0000000e+00],
       [ 1.0916025e-04,  2.1900792e-02, -1.0562437e-01,  1.0000000e+00],
       [ 4.9984737e-04, -7.8452020e-03, -8.7059954e-03,  1.0000000e+00],
       [ 2.9083180e-02,  7.8114352e-02,  1.9575027e-02,  1.0000000e+00],
       [ 0.0000000e+00,  0.0000000e+00,  1.0000000e-01,  1.0000000e+00],
       [ 4.1410813e-04, -2.0290376e-02,  4.5943169e-02,  1.0000000e+00],
       [ 2.9078800e-05, -1.1406515e-01,  1.6447191e-02,  1.0000000e+00],
       [-3.0257825e-05, -1.6511890e-02,  1.1329461e-01,  1.0000000e+00],
       [-3.0879429e-05, -1.1374485e-01, -1.6369065e-02,  1.0000000e+00],
       [-1.0916025e-04,  2.1900792e-02,  1.0562437e-01,  1.0000000e+00],
       [ 4.9984737e-04, -7.8452020e-03,  8.7059954e-03,  1.0000000e+00],
       [ 2.9083180e-02,  7.8114352e-02, -1.9575027e-02,  1.0000000e+00],
       [ 0.0000000e+00,  0.0000000e+00,  1.0000000e-01,  1.0000000e+00],
       [ 1.0000000e-02, -1.4000000e-01,  0.0000000e+00,  1.0000000e+00]])


void get_torques(f, torque_frame_idx, frame_frame_influence,
                 gravity=None,
                 mode='f'){
    // Finding the torque about a given axis does not depend on the specific location on the axis where the torque acts

//    if gravity is None:
//        gravity = np.array([[0., 0., -9.81]])  # matrix / s**2

    force = mass[np.newaxis, :, np.newaxis] * gravity[:, np.newaxis, :]
    x_mass = frames2pos(f=f, f_idx=mass_frame_idx, x_rel=mass_pos)

    torques_around_point = np.empty(shape + [len(torque_frame_idx), 3])
    for i, idx_frame in enumerate(torque_frame_idx):
        x_frame = f[..., idx_frame, :3, -1]
        mass_bool = frame_frame_influence[idx_frame][mass_frame_idx]
        r = x_mass[..., mass_bool, :] - x_frame[..., np.newaxis, :]
        torques_around_point[..., i, :] = np.cross(r, force[:, mass_bool, :]).sum(axis=-2)

    if mode == 'dh':
        # IDENTIFICATION OF GEOMETRIC AND NON GEOMETRIC PARAMETERS OF ROBOTS, J.L. Caenen, J.C. Ange, 1990
        # Torque_x = (0_M_j - 0_P_(i-1)) x (m_j*g) @ x_(i-1)
        # Torque_y = (0_M_j - 0_P_(i-1)) x (m_j*g) @ y_(i-1)
        # Torque_z = (0_M_j - 0_P_i)     x (m_j*g) @ z_i
        f_rot_xy = np.swapaxes(f[..., torque_frame_idx-1, :3, 0:2], -1, -2)
        f_rot_z = np.swapaxes(f[..., torque_frame_idx, :3, 2:3], -1, -2)
        f_rot = np.concatenate((f_rot_xy, f_rot_z), axis=-2)

    elif mode == 'f':
        f_rot = np.swapaxes(f[..., torque_frame_idx, :3, :3], -1, -2)
    else:
        raise ValueError(f"Unknown mode {mode}")
    torques_around_axes = (f_rot @ torques_around_point[..., np.newaxis])[..., 0]

    return torques_around_point, torques_around_axes


                 }


def torque_compliance2dh(torque, dh, el, include_beta=False):
    if torque is None or el is None:
        return None

    dh_trq = torque * el  # x, y, z
    if include_beta:
        dh2 = np.zeros((dh_trq.shape[0], dh.shape[0], 5))
        dh2[..., :dh.shape[-1]] = dh
        dh2[..., [3, 4, 1]] += dh_trq  # x, y, z
    else:
        dh2 = np.zeros((dh_trq.shape[0], dh.shape[0], 5))
        dh2[..., :dh.shape[-1]] = dh
        dh2[..., [3, 1]] += dh_trq[..., [0, 2]]  # x, z
    return dh2