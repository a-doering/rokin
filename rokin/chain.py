import numpy as np
from wzk import element_at_depth


def next_frame_idx2influence_frames_frames(nfi):
    n_frames = len(nfi)
    assert nfi[-1] == -1
    iff = np.eye(n_frames, dtype=bool)  # influence frame-> frame
    for frame in range(n_frames - 2, -1, -1):
        next_frame = nfi[frame]
        if isinstance(next_frame, (list, tuple)):
            for nf in next_frame:
                iff[frame] += iff[nf]

        elif next_frame != -1:
            iff[frame] += iff[next_frame]

    return iff


def influence_frames_frames2joints_frames(jfi, iff=None, nfi=None):
    n_dof = len(jfi)
    if iff is None:
        assert nfi is not None
        iff = next_frame_idx2influence_frames_frames(nfi)

    n_frames = iff.shape[0]

    ijf = np.zeros((n_dof, n_frames), dtype=bool)
    for joint in range(n_dof):
        frame = jfi[joint]
        if isinstance(frame, (list, tuple)):
            for f in frame:
                ijf[joint] += iff[f]
        else:
            ijf[joint] += iff[frame]

    return ijf


def __get_joint_frame_indices_first_last(jfi):
    jfi_first = np.array([i[0] if isinstance(i, (list, tuple, np.ndarray)) else i for i in jfi])
    jfi_last = np.array([i[-1] if isinstance(i, (list, tuple, np.ndarray)) else i for i in jfi])
    return jfi, jfi_first, jfi_last


def prev2next_frame_idx(pfi):
    pfi = np.array(pfi)
    nfi = np.zeros_like(pfi, dtype=object)
    for i in range(len(pfi)):
        where_i = np.nonzero(pfi == i)[0]
        if len(where_i) == 0:
            nfi[i] = -1
        elif len(where_i) == 1:
            nfi[i] = where_i[0]
        else:  # len(where_i) > 1
            nfi[i] = where_i.tolist()

    return nfi


def next2prev_frame_idx(nfi):
    nfi = nfi.tolist()
    nfi_level1 = element_at_depth(nfi, d=1, with_index=True)
    pfi = np.zeros(len(nfi), dtype=int)
    pfi[0] = -1

    for i in range(1, len(pfi)):
        try:
            pfi[i] = nfi.index(i)
        except ValueError:
            for j, nfi_l1 in nfi_level1:
                if i in nfi_l1:
                    pfi[i] = j[0]
                    continue

    return pfi


# fill
def complete_chain_parameters(robot):
    try:
        robot.prev_frame_idx = next2prev_frame_idx(nfi=robot.next_frame_idx)

    except AttributeError:
        robot.next_frame_idx = prev2next_frame_idx(pfi=robot.prev_frame_idx)

    robot.frame_frame_influence = next_frame_idx2influence_frames_frames(nfi=robot.next_frame_idx)
    robot.joint_frame_influence = influence_frames_frames2joints_frames(jfi=robot.joint_frame_idx,
                                                                        iff=robot.frame_frame_influence,
                                                                        nfi=robot.next_frame_idx)


#
def in_kinematic_chain(jf_influence, f_idx):
    return jf_influence[:, f_idx].sum(axis=-1) != 0


def not_in_kinematic_chain(influence_jf, f_idx):
    return np.logical_not(in_kinematic_chain(jf_influence=influence_jf, f_idx=f_idx))


def num_in_kinematic_chain(influence_jf, f_idx):
    return sum(in_kinematic_chain(jf_influence=influence_jf, f_idx=f_idx))


def replace_outside_kinematic_chain(q, q2, in_kc):
    n_dof = len(in_kc)
    n_samples_q, n_wp, n_dof_q = q.shape
    q_new = np.zeros((n_samples_q, n_wp, n_dof))

    if q2 is None:
        if n_dof_q < n_dof:
            q_new[..., in_kc] = q
        else:
            q_new = q
    else:
        q_new[:] = q2
        if n_dof_q < n_dof:
            q_new[..., in_kc] = q
        else:
            q_new[..., in_kc] = q[..., in_kc]

    return q_new


def shift_pfi(pfi, shift):

    for i, pfi_i in enumerate(pfi):
        if pfi_i != -1:
            pfi[i] += shift
    return pfi


def shift_nfi(nfi, shift):
    for i, nfi_i in enumerate(nfi):
        if isinstance(nfi_i, list):
            for j, nfi_ii in enumerate(nfi_i):
                if nfi_ii != -1:
                    nfi[i][j] += shift
        elif nfi_i != -1:
            nfi[i] += shift
    return nfi


def combine_chains_end(nfi_a, nfi_b, i=0):
    assert nfi_a[i] == -1
    nfi_a[i] = len(nfi_a)
    nfi_b = shift_nfi(nfi_b, len(nfi_a))
    nfi = np.hstack((nfi_a, nfi_b))
    return nfi


def combine_chains(nfi_list, mode='base'):
    if len(nfi_list) == 1:
        return nfi_list

    if mode == 'base':
        cs = np.cumsum([1] + [len(nfi) for nfi in nfi_list[:-1]])

        nfi_final = [cs.tolist()]
        for nfi in nfi_list:
            nfi = shift_nfi(nfi=np.copy(nfi), shift=len(nfi_final))
            nfi_final += nfi.tolist()

        return np.array(nfi_final, dtype=object)

    elif mode == 'end':

        nfi_final = nfi_list[0]
        for nfi in nfi_list[1:]:
            nfi = (shift_nfi(nfi=np.copy(nfi), shift=len(nfi_final)))
            nfi_final = np.hstack((nfi_final, nfi))

        return nfi_final

    else:
        raise ValueError
