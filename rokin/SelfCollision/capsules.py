import numpy as np


def get_capsules_info(capsules):
    capsules_f_idx = []
    capsules_rad = []
    capsules_pos = []
    for i, f in enumerate(capsules):
        for j, c in enumerate(f):
            if c[-1] > 0:
                capsules_f_idx.append(i)
                capsules_pos.append(c[0])
                capsules_rad.append(c[1])

    capsules_frame_idx = np.array(capsules_f_idx)
    capsules_rad = np.array(capsules_rad)
    capsules_pos = np.array(capsules_pos)  # Use as homogeneous coordinates
    capsules_pos = np.concatenate((np.array(capsules_pos), np.ones((len(capsules_pos), 2, 1))), axis=-1)

    return capsules_frame_idx, capsules_pos, capsules_rad
