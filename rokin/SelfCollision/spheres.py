import numpy as np


def get_active_spheres(spheres):
    i = spheres[:, :, -1] > 0
    return spheres[i[:, :, np.newaxis].repeat(4, axis=-1)].reshape(-1, 4)


def get_sphere_frames_active(sphere_radius, n_frames):
    sfa = np.sign(sphere_radius) * np.arange(1, n_frames + 1)[:, np.newaxis]
    return (sfa[sfa > 0] - 1).astype(int)


def get_spheres_info(spheres):
    spheres_f_idx = []
    spheres_list = []
    for i, f in enumerate(spheres):
        for j, s in enumerate(f):
            if s[-1] > 0:
                spheres_f_idx.append(i)
                spheres_list.append(s)

    spheres_f_idx = np.array(spheres_f_idx)
    spheres = np.array(spheres_list)
    spheres_rad = spheres[:, -1]
    spheres_pos = spheres.copy()
    spheres_pos[:, -1] = 1  # Use as homogeneous coordinates

    return spheres_f_idx, spheres_pos, spheres_rad
