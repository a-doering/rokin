import numpy as np
from wzk.spatial import initialize_frames

mesh_folder = '/Users/jote/Documents/DLR/Data/URDFs/Justin19/binary_stl'

# Base
base_meshes = np.array([mesh_folder + '/mobile-body.stl',
                        mesh_folder + '/mobile-steering-motor.stl',
                        mesh_folder + '/mobile-steering-motor.stl',
                        mesh_folder + '/mobile-steering-motor.stl',
                        mesh_folder + '/mobile-steering-motor.stl',
                        mesh_folder + '/mobile-wheel-fork.stl',
                        mesh_folder + '/mobile-wheel-fork.stl',
                        mesh_folder + '/mobile-wheel-fork.stl',
                        mesh_folder + '/mobile-wheel-fork.stl',
                        mesh_folder + '/mobile-wheel.stl',
                        mesh_folder + '/mobile-wheel.stl',
                        mesh_folder + '/mobile-wheel.stl',
                        mesh_folder + '/mobile-wheel.stl'])
base_mesh_frame_idx = np.zeros(len(base_meshes))

# Torso
torso_meshes = np.array([mesh_folder + '/torso-link2.stl',
                         mesh_folder + '/torso-link3.stl',
                         mesh_folder + '/torso-link3.stl',
                         mesh_folder + '/torso-link5.stl'])
torso_mesh_frame_idx = np.array([1, 2, 3, 4])

# Right
right_meshes = np.array([mesh_folder + '/right-lwr-3-link12-34.stl',
                         mesh_folder + '/right-lwr-3-link23-45.stl',
                         mesh_folder + '/right-lwr-3-link12-34.stl',
                         mesh_folder + '/right-lwr-3-link23-45.stl',
                         mesh_folder + '/right-lwr-3-link56.stl',
                         mesh_folder + '/right-lwr-3-link67.stl',
                         mesh_folder + '/right-lwr-3-link7-bsa.stl'])
right_mesh_frame_idx = np.array([1, 2, 3, 4, 5, 6, 7])
right_mesh_frames = initialize_frames(shape=len(right_meshes), n_dim=3, mode='eye')

# Left
left_meshes = np.array([mesh_folder + '/left-lwr-3-link12-34.stl',
                        mesh_folder + '/left-lwr-3-link23-45.stl',
                        mesh_folder + '/left-lwr-3-link12-34.stl',
                        mesh_folder + '/left-lwr-3-link23-45.stl',
                        mesh_folder + '/left-lwr-3-link56.stl',
                        mesh_folder + '/left-lwr-3-link67.stl',
                        mesh_folder + '/left-lwr-3-link7-bsa.stl'])
left_mesh_frame_idx = np.array([1, 2, 3, 4, 5, 6, 7])
left_mesh_frames = initialize_frames(shape=len(left_meshes), n_dim=3, mode='eye')

# Head
head_meshes = np.array([mesh_folder + '/head-link1.stl',
                        mesh_folder + '/head-link2.stl'])
head_mesh_frame_idx = np.array([1, 2])
head_mesh_frames = initialize_frames(shape=len(head_meshes), n_dim=3, mode='eye')
