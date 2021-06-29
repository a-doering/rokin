import numpy as np

from rokin.Robots import Robot, import_robot_cpp
from rokin import chain
from rokin.Robots.Justin19 import justin19_par as jpt
from rokin.Robots.Justin19.spheres import ARM_SPHERES_F_IDX, ARM_SPHERES_POS, ARM_SPHERES_RAD


class JustinArm07(Robot):

    def __init__(self):
        self.id = 'JustinArm07'
        self.n_dim = 3
        self.n_dof = 7
        self.n_frames = jpt.N_FRAMES_RIGHT - 1
        self.limits = jpt.JOINT_LIMITS_RIGHT2
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = jpt.DH_RIGHT
        self.next_frame_idx = np.hstack((np.arange(1, self.n_frames), -1))
        self.joint_frame_idx = np.arange(self.n_dof)
        chain.complete_chain_parameters(robot=self)

        self.f_static = jpt.F_RIGHT_TCP[np.newaxis, :, :]
        self.f_idx_static = np.array([self.n_frames-1])
        self.joint_frame_idx_dh = self.joint_frame_idx
        self. coupled_passive_joints = {}
        self.coupled_passive_joints_jac = {}

        self.spheres_pos = ARM_SPHERES_POS.copy()
        self.spheres_rad = ARM_SPHERES_RAD.copy()
        self.spheres_f_idx = ARM_SPHERES_F_IDX.copy()

        self._cpp = import_robot_cpp(robot=self, replace=False)
