# from /home/baeuml/src/casadi-planner/planner/ajustin-collision.rkt

# Old structure: (27 frames) x (4 spheres per frame)
# New structure: List with 27 frames, each frame with a list of sphere
# Each sphere is described as (x, y, z, r) in its respective frame
# A negative radius means that the sphere is not active
# Do not include base in obstacle check, only if base is capable of motion

import numpy as np

from rokin.SelfCollision import spheres

# This is a newer version (April 2021), where the limits of 4 spheres per frame no longer exist
# Tried to combine the spheres where possible to have lesser frames with more spheres
TORSO_SPHERES = np.array([
    # 0
    [(+0.1, 0, -0.4, 0.3),  # Torso Barrel
     (-0.1, 0, -0.4, 0.3),
     (+0.1, 0, -0.2, 0.3),
     (-0.1, 0, -0.2, 0.3),
     (+0.3, +0.3, -0.4, 0.15),  # FourWheels
     (-0.3, +0.3, -0.4, 0.15),
     (+0.3, -0.3, -0.4, 0.15),
     (-0.3, -0.3, -0.4, 0.15)],
    #   # No FourWheels
    # 1
    [],
    # 2
    [(0.0, 0, 0, 0.14),
     (0.1, 0, 0, 0.14),
     (0.2, 0, 0, 0.14),
     (0.3, 0, 0, 0.14)],
    # 3
    [(0.0, 0, 0, 0.14),
     (0.1, 0, 0, 0.14),
     (0.2, 0, 0, 0.14),
     (0.3, 0, 0, 0.14)],
    # 4
    [(0.0, 0.05, -0.05, 0.14),
     (0.0, 0.05, +0.05, 0.14),
     (0.1, 0.10, -0.05, 0.14),
     (0.1, 0.10, +0.05, 0.14),
     ]
    ], dtype=object)

RIGHT_SPHERES = np.array([
    # 0
    [(0, 0,   -0.08, 0.1)],
    # 1
    [],
    # 2
    [(0, 0.010, 0, 0.120),
     (0, 0.100, 0, 0.085),
     (0, 0.215, 0, 0.090),
     (0, 0.360, 0, 0.115)],
    # 3
    [],
    # 4
    [(0, 0.010, 0, 0.120),
     (0, 0.100, 0, 0.085),
     (0, 0.215, 0, 0.090),
     (0, 0.360, 0, 0.090)],  # r=0.115 FIXME
    # 5
    [],
    # 6
    [],
    # 7
    [],
    # 8
    [(0, 0, -0.09,  0.09),
     (0, 0, 0.030, 0.070),
     (0, 0, 0.075, 0.095),
     (0, 0, 0.170, 0.12),  # with marker: r=0.15, without: r=0.12
     (0, 0, 0.250, 0.070)]
    # [],  # off
    ], dtype=object)

LEFT_SPHERES = np.array([
    # 0
    [(0,  0,   -0.08, 0.1)],
    # 1
    [],
    # 2
    [(0, -0.010, 0, 0.120),
     (0, -0.100, 0, 0.085),
     (0, -0.215, 0, 0.090),
     (0, -0.360, 0, 0.115)],
    # 3
    [],
    # 4
    [(0, -0.010, 0, 0.120),
     (0, -0.100, 0, 0.085),
     (0, -0.215, 0, 0.090),
     (0, -0.360, 0, 0.090)],
    # 5
    [],
    # 6
    [],
    # 7
    [],
    # 8
    [(0, 0, -0.09,  0.09),
     (0,  0, 0.030, 0.070),
     (0,  0, 0.075, 0.095),
     (0,  0, 0.170, 0.120),  # with marker: r=0.15, without: r=0.12
     (0,  0, 0.250, 0.070)],
    # [],  # off
    ], dtype=object)

HEAD_SPHERES = np.array([
    # 0
    [],
    # 1
    [],
    # 2,
    [(+0,     0,   0,  0.05),
     (-0.02, -0.10, 0, 0.08),
     (+0.08, -0.10, 0, 0.10),
     (+0.07, -0.15, 0, 0.10),
     (+0.03, -0.17, 0, 0.13)],  # with marker 0.16, without 0.13
    # 3
    [],
    ], dtype=object)

SPHERES = np.concatenate((TORSO_SPHERES,
                          RIGHT_SPHERES,
                          LEFT_SPHERES,
                          HEAD_SPHERES), axis=0)


SPHERES_F_IDX, SPHERES_POS, SPHERES_RAD = spheres.get_spheres_info(spheres=SPHERES)
ARM_SPHERES_F_IDX, ARM_SPHERES_POS, ARM_SPHERES_RAD = spheres.get_spheres_info(spheres=RIGHT_SPHERES[1:])


def sc_test_arm():
    from rokin.Robots import Justin19
    from rokin.Robots.Justin19.justin19_par import IDX_F_RIGHT

    n = 100000
    robot = Justin19()

    q = robot.sample_q(n)

    x_spheres = robot.get_x_spheres(q=q)

    b = np.array([i in IDX_F_RIGHT for i in robot.spheres_f_idx])

    def get_xiri(i):
        return x_spheres[:, robot.spheres_f_idx == i, :], robot.spheres_rad[robot.spheres_f_idx == i]
    x5, r5 = get_xiri(5)
    x7, r7 = get_xiri(7)
    x13, r13 = get_xiri(13)
    x7 = x_spheres[:, robot.spheres_f_idx == 7, :]

    def get_d(xi, ri, xj, rj):
        d_ij = xi[:, :, np.newaxis, :] - xj[:, np.newaxis, :, :]
        d_ij = np.linalg.norm(d_ij, axis=-1)
        d_ij -= (ri[:, np.newaxis] + rj[np.newaxis, :])
        return d_ij

    d_7_13 = get_d(x7, r7, x13, r13)
    # d_7_13 = d_7_13.min(axis=0)
    d_5_13 = get_d(x5, r5, x13, r13)
    # d_5_13 = d_5_13.min(axis=0)

    b_5_13 = (d_5_13 < 0).sum(axis=(-1, -2)) > 1
    b_7_13 = (d_7_13 < 0).sum(axis=(-1, -2)) > 1

    set1 = set(*np.nonzero(b_5_13))
    set2 = set(*np.nonzero(b_7_13))

    print(len(set1)/n * 100)
    print(len(set2)/n * 100)
    # print(set1.intersection(set2))
    # print(set1.intersection(set2))
    b_7_13.sum()
    b_5_13.sum()
    (b_7_13 * b_5_13).sum()
    print(robot.spheres_f_idx[b])
    x_spheres = x_spheres[:, b, :]
