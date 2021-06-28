# noinspection PyUnresolvedReferences,PyPep8Naming
import rokin.Kinematic.Robots.Justin19_wolfram.Justin19_wolfram as cpp
import numpy as np


def get_frames(q, f_world_robot):
    n_samples, n_wp, n_dof = q.shape
    if f_world_robot is None or np.allclose(f_world_robot, np.eye(4)):
        return cpp.get_frames(q=q, n_samples=n_samples, n_wp=n_wp)
    else:
        return f_world_robot @ cpp.get_frames(q=q, n_samples=n_samples, n_wp=n_wp)


def get_frames_jac(q, f_world_robot):
    n_samples, n_wp, n_dof = q.shape
    frames, frames_jac = cpp.get_frames_jacs(q=q, n_samples=n_samples, n_wp=n_wp)

    if f_world_robot is None or np.allclose(f_world_robot, np.eye(4)):
        return frames, frames_jac
    else:
        return f_world_robot @ frames, frames_jac

