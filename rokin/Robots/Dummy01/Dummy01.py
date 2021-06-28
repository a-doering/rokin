import numpy as np
from rokin.Kinematic.Robots import Robot
import rokin.Kinematic.chain as kc

try:
    # noinspection PyUnresolvedReferences,PyPep8Naming
    from rokin.Kinematic.Robots.Dummy01.cpp import Dummy01 as cpp
except ModuleNotFoundError:
    cpp = None


class Dummy01(Robot):

    def __init__(self):
        self.id = 'Dummy01'
        self._cpp = cpp

        self.n_dim = 3
        self.n_dof = 1
        self.n_frames = 1
        self.limits = np.deg2rad(np.array([[-180, +180]]))
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = np.array([[0, 0, 0, 0]])

        self.next_frame_idx = np.array([-1])
        self.prev_frame_idx = np.array([-1])
        self.joint_frame_idx = np.array([0])
        self.frame_frame_influence = kc.next_frame_idx2influence_frames_frames(nfi=self.next_frame_idx)
        self.joint_frame_influence = kc.influence_frames_frames2joints_frames(jfi=self.joint_frame_idx,
                                                                              iff=self.frame_frame_influence,
                                                                              nfi=self.next_frame_idx)
        self.f_static = None
        self.f_idx_static = np.array([])

        self.joint_frame_idx_dh = self.joint_frame_idx
        self.coupled_passive_joints = None
        self.coupled_passive_joints_jac = None
