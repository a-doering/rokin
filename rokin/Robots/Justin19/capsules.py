import numpy as np

from rokin.SelfCollision import capsules

TORSO_CAPSULES = np.array([
    # 0
    [(np.array([[+0.1, +0.0, -0.4], [-0.1, +0.0, -0.4]]), 0.30),
     (np.array([[+0.1, +0.0, -0.2], [-0.1, +0.0, -0.2]]), 0.30),
     (np.array([[+0.3, +0.3, -0.4], [-0.3, +0.3, -0.4]]), 0.14),
     (np.array([[+0.3, -0.3, -0.4], [-0.3, -0.3, -0.4]]), 0.14)],
    # 1
    [],
    # 2
    [(np.array([[+0.0, +0.0, +0.0], [+0.3, +0.0, +0.0]]), 0.14)],
    # 3
    [(np.array([[+0.0, +0.0, +0.0], [+0.3, +0.0, +0.0]]), 0.14)],
    # 4
    [(np.array([[+0.0, +0.05, -0.05], [+0.0, +0.05, +0.05]]), 0.14),
     (np.array([[+0.1, +0.10, -0.10], [+0.1, +0.10, +0.10]]), 0.14)]
], dtype=object)

RIGHT_CAPSULES = np.array([
    # 0
    [],
    # 1
    [],
    # 2
    [(np.array([[+0.0, +0.01, +0.0], [+0.0, +0.36, +0.0]]), 0.12)],
    # 3
    [],
    # 4
    [(np.array([[+0.0, +0.01, +0.0], [+0.0, +0.36, +0.0]]), 0.12)],
    # 5
    [],
    # 6
    [],
    # 7
    [],
    # [(np.array([[+0.0, +0.01, +0.0], [+0.0, +0.36, +0.0]]), 0.12)],
    # 8
    [(np.array([[+0.0, +0.0, +0.08], [+0.0, +0.0, +0.2]]), 0.12)]
], dtype=object)

LEFT_CAPSULES = np.array([
    # 0
    [],
    # 1
    [],
    # 2
    [(np.array([[+0.0, -0.01, +0.0], [+0.0, -0.36, +0.0]]), 0.12)],
    # 3
    [],
    # 4
    [(np.array([[+0.0, +-.01, +0.0], [+0.0, -0.36, +0.0]]), 0.12)],
    # 5
    [],
    # 6
    [],
    # 7
    [],
    # 8
    # [(np.array([[+0.0, +0.01, +0.0], [+0.0, +0.36, +0.0]]), 0.12)],
    [(np.array([[+0.0, +0.0, +0.08], [+0.0, +0.0, +0.2]]), 0.12)],  # with marker 0.15, without 0.12
], dtype=object)

HEAD_CAPSULES = np.array([
    # 0
    [],
    # 1
    [],
    # 2
    [(np.array([[+0.03, -0.10, +0.0], [+0.03, -0.15, +0.0]]), 0.15)],  # without marker
    # [(np.array([[+0.03, -0.10, +0.0], [+0.03, -0.18, +0.0]]), 0.15)],  # with marker
    # 3
    [],
], dtype=object)

CAPSULES = np.concatenate((TORSO_CAPSULES,
                           RIGHT_CAPSULES,
                           LEFT_CAPSULES,
                           HEAD_CAPSULES), axis=0)

CAPSULES_F_IDX, CAPSULES_POS, CAPSULES_RAD = capsules.get_capsules_info(capsules=CAPSULES)
