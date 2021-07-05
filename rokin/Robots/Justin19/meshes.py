import numpy as np
from wzk.spatial import initialize_frames

mesh_folder = '/Users/jote/Documents/DLR/Data/URDFs/Justin19/binary_stl'

# Base
base_meshes = np.array([mesh_folder + '/mobile-body.stl',
                        # mesh_folder + '/mobile-steering-motor.stl',
                        # mesh_folder + '/mobile-steering-motor.stl',
                        # mesh_folder + '/mobile-steering-motor.stl',
                        # mesh_folder + '/mobile-steering-motor.stl',
                        # mesh_folder + '/mobile-wheel-fork.stl',
                        # mesh_folder + '/mobile-wheel-fork.stl',
                        # mesh_folder + '/mobile-wheel-fork.stl',
                        # mesh_folder + '/mobile-wheel-fork.stl',
                        # mesh_folder + '/mobile-wheel.stl',
                        # mesh_folder + '/mobile-wheel.stl',
                        # mesh_folder + '/mobile-wheel.stl',
                        # mesh_folder + '/mobile-wheel.stl'
                        ])
base_meshes_f_idx = np.zeros(len(base_meshes), dtype=int)
base_meshes_f = initialize_frames(shape=len(base_meshes), n_dim=3, mode='eye')
base_meshes_f[0] = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, -0.5884],  # measured from base plate which is approx. 10cm above the floor
                             [0, 0, 0, 1]])
# Torso
torso_meshes = np.array([mesh_folder + '/torso-link2.stl',
                         mesh_folder + '/torso-link3.stl',
                         mesh_folder + '/torso-link3.stl',
                         mesh_folder + '/torso-link5.stl'])
torso_meshes_f_idx = np.array([1, 2, 3, 4], dtype=int)
torso_meshes_f = initialize_frames(shape=len(torso_meshes), n_dim=3, mode='eye')

# Right
right_meshes = np.array([mesh_folder + '/right-lwr-3-link12-34.stl',
                         mesh_folder + '/right-lwr-3-link23-45.stl',
                         mesh_folder + '/right-lwr-3-link12-34.stl',
                         mesh_folder + '/right-lwr-3-link23-45.stl',
                         mesh_folder + '/right-lwr-3-link56.stl',
                         mesh_folder + '/right-lwr-3-link67.stl',
                         mesh_folder + '/right-lwr-3-link7-bsa.stl'])
right_meshes_f_idx = np.array([1, 2, 3, 4, 5, 6, 7], dtype=int)
right_meshes_f = initialize_frames(shape=len(right_meshes), n_dim=3, mode='eye')

# Left
left_meshes = np.array([mesh_folder + '/left-lwr-3-link12-34.stl',
                        mesh_folder + '/left-lwr-3-link23-45.stl',
                        mesh_folder + '/left-lwr-3-link12-34.stl',
                        mesh_folder + '/left-lwr-3-link23-45.stl',
                        mesh_folder + '/left-lwr-3-link56.stl',
                        mesh_folder + '/left-lwr-3-link67.stl',
                        mesh_folder + '/left-lwr-3-link7-bsa.stl'])
left_meshes_f_idx = np.array([1, 2, 3, 4, 5, 6, 7], dtype=int)
left_meshes_f = initialize_frames(shape=len(left_meshes), n_dim=3, mode='eye')

# Head
head_meshes = np.array([mesh_folder + '/head-link1.stl',
                        mesh_folder + '/head-link2.stl'])
head_meshes_f_idx = np.array([1, 2], dtype=int)
head_meshes_f = initialize_frames(shape=len(head_meshes), n_dim=3, mode='eye')

# Justin
MESHES = np.concatenate((base_meshes, torso_meshes, right_meshes, left_meshes, head_meshes), axis=0)
MESHES_F = np.concatenate((base_meshes_f, torso_meshes_f,
                           right_meshes_f, left_meshes_f,
                           head_meshes_f), axis=0)
MESHES_F_IDX = np.concatenate((base_meshes_f_idx+0, torso_meshes_f_idx+0,
                               right_meshes_f_idx+5, left_meshes_f_idx+14,
                               head_meshes_f_idx+23), dtype=int, axis=0)
