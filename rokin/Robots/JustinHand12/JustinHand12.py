import numpy as np
from wzk.spatial import trans_rotvec2frame
from wzk.numpy2 import tile_offset

from rokin import dh, chain
from rokin.Robots import Robot, JustinFinger03

try:
    # noinspection PyUnresolvedReferences,PyPep8Naming
    from rokin.Robots.JustinHand12.cpp import JustinHand12 as cpp
except ModuleNotFoundError:
    cpp = None
except ImportError:
    cpp = None

_finger03 = JustinFinger03()


class JustinHand12(Robot):
    def __init__(self):
        self.id = 'JustinHand12'
        self._cpp = cpp

        self.n_dim = 3
        self.n_dof = _finger03.n_dof * 4
        self.n_frames = 1 + _finger03.n_frames*4
        self.limits = np.tile(_finger03.limits, reps=(4, 1))
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = np.tile(_finger03.dh, reps=(4, 1))

        self.next_frame_idx = chain.combine_chains(nfi_list=[_finger03.next_frame_idx]*4, mode='base')
        self.next_frame_idx = chain.combine_chains_end(nfi_a=np.array([-1]), nfi_b=self.next_frame_idx, i=0)
        self.joint_frame_idx = np.array([1, 2, [3, 4],
                                         7, 8, [9, 10],
                                         13, 14, [15, 16],
                                         19, 20, [21, 22]], dtype='object')
        chain.complete_chain_parameters(robot=self)

        self.f_static = np.zeros((9, 4, 4))
        self.f_static[0] = np.eye(4)
        self.f_static[2::2] = dh.frame_from_dh(q=0, d=0.029, theta=np.pi, a=0, alpha=-np.pi/2)[np.newaxis, :, :]

        # Note the inverted ordering: ring, middle, fore, thumb
        # Ring
        self.f_static[1] = trans_rotvec2frame(trans=np.array([-0.005785206, -0.061149354, +0.132621405]),
                                              rotvec=np.array([-1.63850239, -0.74407776, -1.79289304]))
        # Middle
        self.f_static[3] = trans_rotvec2frame(trans=np.array([-0.048151365, -0.012101809, +0.133300263]),
                                              rotvec=np.array([-1.90822554, +0.04794712, -2.36863463]))
        # Fore
        self.f_static[5] = trans_rotvec2frame(trans=np.array([-0.045982316, +0.027691585,  +0.138843566]),
                                              rotvec=np.array([-1.90822554,  0.04794712, -2.36863463]))
        # Thumb
        self.f_static[7] = trans_rotvec2frame(trans=np.array([+0.052264236, +0.026986124, +0.102620128]),
                                              rotvec=np.array([+0.20806912, -1.05993301, -0.25277084]))

        self.f_idx_static = np.hstack(([0], 1 + tile_offset(np.array([0, 5]), reps=4, offsets=_finger03.n_frames)))
        self.joint_frame_idx_dh = 1 + tile_offset(_finger03.joint_frame_idx_dh, reps=4, offsets=_finger03.n_frames)
        self.coupled_passive_joints = {3: lambda q: q[2],
                                       7: lambda q: q[5],
                                       11: lambda q: q[8],
                                       15: lambda q: q[11]}
        self.coupled_passive_joints_jac = {3: lambda q:  np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                                           7: lambda q:  np.array([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]),
                                           11: lambda q: np.array([0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]),
                                           15: lambda q: np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])}

        # Spheres
        self.spheres_pos = np.tile(_finger03.spheres_pos, (4, 1))
        self.spheres_rad = np.tile(_finger03.spheres_rad, 4)
        self.spheres_f_idx = 1 + tile_offset(a=_finger03.spheres_f_idx, reps=4, offsets=_finger03.n_frames)

        # Capsules
        self.capsules_pos = np.tile(_finger03.capsules_pos, (4, 1, 1))
        self.capsules_rad = np.tile(_finger03.capsules_rad, 4)
        self.capsules_f_idx = 1 + tile_offset(a=_finger03.capsules_f_idx, reps=4, offsets=_finger03.n_frames)

        # Meshes
        mesh_folder = '/Users/jote/Documents/DLR/Data/URDFs/JustinHand12/hand_generator/resources/hand_model/binary_stl'
        self.meshes = np.array([mesh_folder + '/full_hand_base.stl'] +
                               [mesh_folder + '/finger_base.stl',
                                mesh_folder + '/finger_prox.stl',
                                mesh_folder + '/finger_med.stl',
                                mesh_folder + '/finger_dist_pill.stl']*4)

        trans = np.zeros((1+4*4, 3))
        rotvec = np.zeros((1+4*4, 3))
        rotvec[1:, 0] = -np.pi/2
        self.meshes_frames = trans_rotvec2frame(trans=trans, rotvec=rotvec)

        self.meshes_f_idx = np.concatenate([[0], 1 + tile_offset(a=np.array([0, 2, 3, 4]), reps=4,
                                                                 offsets=_finger03.n_frames)])


# cal_rob.next_frame_idx
# array([
# 1,
# 2,  3,  4,  5,  6, -1,
# 8,  9, 10, 11, 12, -1,
# 14, 15, 16, 17, 18, -1,
# 20, 21, 22, 23, 24, -1])

# cal_rob.capsules_f_idx
# array([
# 3,  4,  5,
# 9, 10, 11,
# 15, 16, 17,
# 21, 22, 23])
