import numpy as np

from rokin.Robots.JustinBase03.justinbase03_par import WHEEL_MOUNTING_RADII, WHEEL_MOUNTING_ANGLES

from wzk.spatial.transform_2d import trans_theta2frame
"""Return everything in radians"""


def wheels_pos_global(xq, wheel_pos):
    f_wheel_base = trans_theta2frame(trans=xq[..., :2], theta=xq[..., 0])
    x_wheels = f_wheel_base @ wheel_pos
    return x_wheels


def looped_square():
    n = 400
    xq = np.zeros((n, 3))
    return xq


def turn_around_point(xq0, point, n=360, theta=+2 * np.pi):

    diff = xq0[:2] - point
    theta0 = np.arctan2(diff[1], diff[0])
    radius = np.linalg.norm(diff)

    theta = np.linspace(0, theta, num=n)

    xq = np.zeros((n+1, 3))
    xq[:, :] = xq0
    xq[1:, 0] += np.cos(theta0 + theta) * radius
    xq[1:, 1] += np.sin(theta0 + theta) * radius
    xq[1:, 2] += theta


def create_sample_trajectories(mode='d'):

    if mode == 'd':
        n = 360
        xq = np.zeros((3 * n, 3))
        xq[:n, 0] = 3 + WHEEL_MOUNTING_RADII[2]
        xq[:n, 1] = 3
        xq[:n, 2] = -(np.pi + WHEEL_MOUNTING_ANGLES[2])

        xq[n:2 * n, 0] = 3 + np.cos(np.linspace(0, 2*np.pi, num=n)) * WHEEL_MOUNTING_RADII[2]
        xq[n:2 * n, 1] = 3 + np.sin(np.linspace(0, 2*np.pi, num=n)) * WHEEL_MOUNTING_RADII[2]
        xq[n:2 * n, 2] = -(np.pi + WHEEL_MOUNTING_ANGLES[2]) + np.linspace(0, 2 * np.pi, num=n)
        # xq[:n, 2] = 0

        xq[2 * n:, 0] = 3 + WHEEL_MOUNTING_RADII[2]
        xq[2 * n:, 1] = np.linspace(start=3, stop=6, num=n)
        xq[2 * n:, 2] = -(np.pi + WHEEL_MOUNTING_ANGLES[2])
        xq = xq[n:]
        return xq

    elif mode == 'arc1':
        n = 200
        xq = np.zeros((n, 3))
        xq[:, 0] = np.sin(np.linspace(0, np.pi, n)) * 3
        xq[:, 1] = np.linspace(1, 10, n)
        xq[:, 2] = np.linspace(0, np.pi, n)
        return xq

    elif mode == 'arc2':
        n = 10
        xq = np.zeros((n, 3))
        xq[:, 0] = np.sin(np.linspace(0, np.pi, n)) * 3
        xq[:, 1] = np.linspace(1, 10, n)
        xq[:, 2] = 0
        return xq

    elif mode == 'circle_xy':
        n = 10
        xq = np.zeros((n, 3))
        xq[:, 0] = 4 + np.cos(np.linspace(0, 2*np.pi, n)) * 3
        xq[:, 1] = 4 + np.sin(np.linspace(0, 2*np.pi, n)) * 3
        xq[:, 2] = 0
        return xq

    elif mode == 'circle_xy_theta':
        n = 10
        xq = np.zeros((n, 3))
        xq[:, 0] = 4 + np.cos(np.linspace(0, 2*np.pi, n)) * 3
        xq[:, 1] = 4 + np.sin(np.linspace(0, 2*np.pi, n)) * 3
        xq[:, 2] = np.linspace(0, 2*np.pi, n)
        return xq

    elif mode == 'circle_theta':
        n = 720
        xq = np.zeros((n, 3))
        xq[:, 0] = 4
        xq[:, 1] = 4
        xq[:, 2] = np.linspace(0, 2*np.pi, n)
        return xq

    elif mode == 'screw':
        n = 720
        xq = np.zeros((n, 3))
        xq[:, 0] = np.linspace(1, 5, n)
        xq[:, 1] = 4
        xq[:, 2] = np.linspace(0, 2*np.pi, n)
        return xq

    elif mode == 'peak':
        n = 40

        xq = np.zeros((n, 3))
        xq[:, 0] = np.linspace(1, 5, n)
        xq[:, 1] = np.linspace(1, 5, n)
        xq[n//2:, 1] = 2 - xq[n//2:, 1]
        return xq

    elif mode == 'random_xy':
        n = 5
        xq = np.zeros((n, 3))
        xq[0, :2] = (1, 1)
        xq[-1, :2] = (9, 9)
        xq[1:-1, :2] = np.random.random((n-2, 2)) * 5
        return xq

    elif mode == 'random_xy_theta':
        n = 5
        xq = np.zeros((n, 3))
        xq[0, :2] = (1, 1)
        xq[-1, :2] = (9, 9)
        xq[1:-1, :2] = np.random.random((n-2, 2)) * 10
        xq[1:-1, 2] = np.random.random((n-2)) * np.pi*2
        return xq
