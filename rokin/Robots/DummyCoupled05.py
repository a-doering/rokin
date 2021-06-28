import numpy as np


class DummyCoupled05:
    def __init__(self):

        self.id = 'DummyCoupled05'
        self.n_dim = 3
        self.n_dof = 2
        self.n_frames = 5
        self.limits = None
        self.infinity_joints = np.zeros(self.n_dof, dtype=bool)
        self.f_world_robot = np.eye(4)

        # Kinematic Chain
        self.dh = np.array([[0, 0, 0.2, 0],
                            [0, 0, 0.4, 0],
                            [0, 0, 0.2, 0],
                            [0, 0, 0.4, 0],
                            [0, 0, 0.6, 0]])
        self.next_frame_idx = np.array([1, 2, 3, 4, -1])
        self.joint_frame_idx = [[0, 2], [1, 3, 4]]

        self.joint_frame_idx_dh = [0, 1, 2, 3, 4]
        self.coupled_passive_joints = {2: lambda q: 2*q[0],
                                       3: lambda q: 2*q[1],
                                       4: lambda q: q[0]+q[1],
                                       }

        self.f_idx_static = None
        self.f_static = None


# justin19 = Justin19()
# justin19.get_frames(np.random.random((1, 19)))
