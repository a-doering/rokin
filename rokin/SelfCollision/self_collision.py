import numpy as np


def get_collision_matrix_frames(n,
                                exclude_ranges, exclude_pairs, include_pairs):
    """
    create boolean collision matrices which indicates which frame pairs i, j are collidable
    Additional assumption of no self collision for i == j

    exclude_range: no checks between frames on the same body part defined by a range of frames
    exclude_pairs: no checks between those specific pairs of frames
    include_pairs: checks between those specific pairs of frames, is applied after exclude range to allow for exceptions
    AssertionError when exclude_pairs and include_pairs contradict
    """
    assert len(set(exclude_pairs).intersection(set(include_pairs))) == 0

    cm = ~np.eye(n, dtype=bool)

    # exclude_ranges
    for r in exclude_ranges:
        r = np.array(r)
        cm[r[:, np.newaxis], r[np.newaxis, :]] = False

    # exclude_pairs
    for (i, j) in exclude_pairs:
        cm[i, j] = cm[j, i] = False

    # include_pairs
    for (i, j) in include_pairs:
        cm[i, j] = cm[j, i] = True

    return cm


def get_collision_matrix_objects(cm_frames, objects_f_idx):
    n = len(objects_f_idx)
    cm_objects = np.ones((n, n), dtype=bool)
    for ii in range(n):
        fi = objects_f_idx[ii]
        for jj in range(n):
            fj = objects_f_idx[jj]
            cm_objects[ii, jj] = cm_objects[jj, ii] = cm_frames[fi, fj]

    return cm_objects


def collision_matrix2pairs(cm):
    return np.array(np.nonzero(np.triu(cm, k=0))).T


def get_collision_objects(robot, mode='spheres'):
    if mode == 'spheres':
        objects_f_idx = robot.spheres_f_idx
        objects_rad = robot.spheres_rad

    elif mode == 'capsules':
        objects_f_idx = robot.capsules_f_idx
        objects_rad = robot.capsules_rad

    elif mode == 'gjkepa':
        raise NotImplementedError
    else:
        raise ValueError

    return objects_f_idx, objects_rad


def initialize(sc, robot, mode='spheres'):
    try:
        cm_frames = robot.self_collision_matrix
    except AttributeError:
        return None

    if cm_frames is None:
        return None

    objects_f_idx, objects_rad = get_collision_objects(robot=robot, mode=mode)
    cm_capsules = get_collision_matrix_objects(cm_frames=cm_frames, objects_f_idx=objects_f_idx)

    sc.collision_pairs = collision_matrix2pairs(cm_capsules)
    sc.radii_combined = objects_rad[sc.collision_pairs].sum(axis=1)

    return sc


def update(sc, cm_frames, object_f_idx, object_rad):
    cm_spheres = get_collision_matrix_objects(cm_frames=cm_frames, objects_f_idx=object_f_idx)
    sc.collision_pairs = collision_matrix2pairs(cm_spheres)
    sc.n_collision_pairs = sc.collision_pairs.shape[0]

    # Get the combined radii for each object pair r0 + r1
    sc.radii_combined = object_rad[sc.collision_pairs].sum(axis=1)
    return sc


def get_collision_info(collision_pairs, i):
    return f"({collision_pairs[i][0]})<-->({collision_pairs[i][1]})"
