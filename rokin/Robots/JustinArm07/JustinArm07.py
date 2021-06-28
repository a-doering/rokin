import numpy as np
from rokin.Kinematic.Robots import Robot
import rokin.Kinematic.chain as kc

from mopla.Justin import parameter_torso as jpt
from mopla.Justin.spheres import ARM_SPHERES_F_IDX, ARM_SPHERES_POS, ARM_SPHERES_RAD

try:
    # noinspection PyUnresolvedReferences,PyPep8Naming
    from rokin.Kinematic.Robots.JustinArm07.cpp import JustinArm07 as cpp
except ModuleNotFoundError:
    cpp = None


class JustinArm07(Robot):

    def __init__(self):
        self.id = 'JustinArm07'
        self._cpp = cpp
        self.n_dim = 3
        self.n_dof = 7
        self.n_frames = jpt.N_FRAMES_RIGHT - 1
        self.limits = jpt.JOINT_LIMITS_RIGHT2
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = jpt.DH_RIGHT
        self.next_frame_idx = np.hstack((np.arange(1, self.n_frames), -1))
        self.prev_frame_idx = np.hstack((-1, np.arange(0, self.n_frames-1)))
        self.joint_frame_idx = np.arange(self.n_dof)
        self.frame_frame_influence = kc.next_frame_idx2influence_frames_frames(nfi=self.next_frame_idx)
        self.joint_frame_influence = kc.influence_frames_frames2joints_frames(jfi=self.joint_frame_idx,
                                                                              iff=self.frame_frame_influence,
                                                                              nfi=self.next_frame_idx)

        self.f_static = jpt.F_RIGHT_TCP[np.newaxis, :, :]
        self.f_idx_static = np.array([self.n_frames-1])
        self.joint_frame_idx_dh = self.joint_frame_idx
        self. coupled_passive_joints = {}
        self.coupled_passive_joints_jac = {}

        self.spheres_pos = ARM_SPHERES_POS.copy()
        self.spheres_rad = ARM_SPHERES_RAD.copy()
        self.spheres_frame_idx = ARM_SPHERES_F_IDX.copy()
