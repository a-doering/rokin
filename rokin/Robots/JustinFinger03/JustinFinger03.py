import numpy as np
from wzk.spatial.transform import trans_rotvec2frame

from rokin import dh, chain
from rokin.Robots import Robot, import_robot_cpp
from rokin.SelfCollision.capsules import get_capsules_info


class JustinFinger03(Robot):

    def __init__(self):
        self.id = 'JustinFinger03'

        self.n_dim = 3
        self.n_dof = 3
        self.n_frames = 6
        self.limits = np.deg2rad(np.array([[-30, +30],
                                           [-20, +86],
                                           [-8, +96]]))  # [-10, +105]] in reality the limits are smaller
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = np.array([[0, 0, 0, 0],
                            [0, 0, 0, np.pi/2],
                            [0, 0, 0.075, 0],
                            [0, -np.pi/2, 0.04, 0]])

        self.next_frame_idx = np.array([1, 2, 3, 4, 5, -1])
        self.joint_frame_idx = np.array([1, 2, [3, 4]], dtype='object')
        self.joint_frame_idx_dh = np.array([1, 2, 3, 4])

        chain.complete_chain_parameters(robot=self)

        self.f_static = np.zeros((2, 4, 4,))
        self.f_static[0] = np.eye(4)
        self.f_static[1] = dh.frame_from_dh(q=0, d=0.029, theta=np.pi, a=0, alpha=-np.pi/2)
        self.f_idx_static = np.array([0, 5])
        self.coupled_passive_joints = {3: lambda q: q[2]}
        self.coupled_passive_joints_jac = {3: lambda q: np.array([0, 0, 1])}

        self.spheres_pos = np.array([[0.075*1/3, 0, 0, 1],
                                     [0.075*2/3, 0, 0, 1],
                                     [0, 0, 0, 1],
                                     [0.02, 0, 0, 1],
                                     [0.04, 0, 0, 1],
                                     [0, 0.0165, 0, 1],
                                     [0, 0.033, 0, 1]])

        self.spheres_rad = np.hstack([[0.022]*2,
                                      [0.018]*3,
                                      [0.01425]*2])
        self.spheres_f_idx = np.hstack([[2]*2,
                                        [3]*3,
                                        [4]*2])

        capsules = np.array([[], [],
                             [(np.array([[+0.075*1/3, 0, 0], [+0.075*2/3, 0, 0]]), 0.022)],   # 2
                             [(np.array([[0, 0, 0], [0.04, 0, +0]]), 0.018)],                 # 3
                             [(np.array([[0, +0.0165, 0], [+0.0, +0.033, +0.0]]), 0.01425)],  # 4
                             []], dtype=object)
        self.capsules_f_idx, self.capsules_pos, self.capsules_rad = get_capsules_info(capsules=capsules)

        mesh_folder = '/Users/jote/Documents/DLR/Data/URDFs/JustinHand12/hand_generator/resources/hand_model/binary_stl'
        self.meshes = np.array([mesh_folder + '/finger_prox.stl',
                                mesh_folder + '/finger_med.stl',
                                mesh_folder + '/finger_dist_pill.stl'])

        # xyz, rpy  # as it is done in URDF files
        self.meshes_frames = trans_rotvec2frame(trans=np.zeros((3, 3)), rotvec=np.array([[-np.pi/2, 0, 0],
                                                                                         [-np.pi/2, 0, 0],
                                                                                         [-np.pi/2, 0, 0]]))

        self.meshes_f_idx = np.array([2, 3, 4])

        self._cpp = import_robot_cpp(robot=self, replace=False)
