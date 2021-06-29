import re
import numpy as np

from wzk.strings import brackets_rir, brackets_sis, brackets_rijr, brackets_sissjs

from rokin import chain

name_joint = 'q'
name_frame = 'f'
name_dict = 'd'
name_jac = 'j'
name_dh = 'dh'
name_dh5 = ['dh_d', 'dh_theta', 'dh_a', 'dh_alpha', 'dh_beta']


comment = '// '
matmul = ' * '
nl = ';\n'

cpp_joint_i = f'{name_joint}' + brackets_sis
cpp_frame_i = f'{name_frame}' + brackets_rir
cpp_jac_ij = f'{name_jac}' + brackets_rijr
cpp_dict_ij = f'{name_dict}' + brackets_rijr

cpp_dh_ij = f'{name_dh}' + brackets_sissjs


math_dict = dict(sin='std::sin', cos='std::cos')
dh_dict = {'dh_d{i}':      'dh[{i}][0]',
           'dh_theta{i}':  'dh[{i}][1]',
           'dh_a{i}':      'dh[{i}][2]',
           'dh_alpha{i}':  'dh[{i}][3]'}


def idx(*, i, j=None):
    if j is None:
        return brackets_rir.format(i=i)
    else:
        return brackets_rijr.format(i=i, j=j)


def var(v, i, j=None):
    return f"{v}{idx(i=i, j=j)}"


def fill_frames_str(frames):

    s = ""
    for i, f in enumerate(frames):
        s += frame2str_eigen(name=var(name_frame, i=i), f=f)

    return s


def fill_jacs_str(frames_jac):

    s = ""
    for i in range(frames_jac.shape[0]):
        for j in range(frames_jac.shape[1]):
            if frames_jac[i, j] is not None:
                s += frame2str_eigen(name=var(name_jac, i=i, j=j), f=frames_jac[i, j])
    return s


def combine_frames_str(nfi):
    pfi = chain.next2prev_frame_idx(nfi)
    s = ''
    for i, pfi_i in enumerate(pfi[1:], start=1):
        s += f"{var(name_frame, i=i)} = {var(name_frame, i=pfi_i)}{matmul}{var(name_frame, i=i)}{nl}"
    return s


def combine_dict_str(nfi):

    n_frames = len(nfi)
    iff = chain.next_frame_idx2influence_frames_frames(nfi=nfi)

    s = ''
    for i in range(n_frames - 1, -1, -1):
        nfi_i = nfi[i]
        iff_i = np.nonzero(iff[i])[0]

        s += f"{comment}{i}\n"
        s += f"{var(name_dict, i=i, j=i)} = {var(name_frame, i=i)}{nl}"

        if isinstance(nfi_i, (list, tuple)):
            nf_i_all = [nf_i2 for nf_i2 in nfi_i for _ in range(iff[nf_i2].sum())]
            for iff_ij, nf_i2 in zip(iff_i[1:], nf_i_all):
                s += f"{var(name_dict, i=i, j=iff_ij)} = " \
                     f"{var(name_frame, i=i)}{matmul}{var(name_dict, i=nf_i2, j=iff_ij)}{nl}"

        else:
            for iff_ij in iff_i[1:]:
                s += f"{var(name_dict, i=i, j=iff_ij)} = " \
                     f"{var(name_frame, i=i)}{matmul}{var(name_dict, i=nfi_i, j=iff_ij)};\n"

    return s


def combine_jacs_str(nfi, joint_frame_idx):

    # n_frames = len(nfi)
    n_dof = len(joint_frame_idx)

    pfi = chain.next2prev_frame_idx(nfi=nfi)
    iff = chain.next_frame_idx2influence_frames_frames(nfi=nfi)
    ijf = chain.influence_frames_frames2joints_frames(jfi=joint_frame_idx, iff=iff)

    jf_all, jf_first, jf_last = chain.__get_joint_frame_indices_first_last(joint_frame_idx)

    pfi_ = pfi[jf_first]
    jf_first_ = jf_first  # [pfi_ != -1]
    pfi_ = pfi_  # [pfi_ != -1]

    s = ''
    for i in range(n_dof):
        s += f"{comment}{i}\n"
        if pfi_[i] != -1:
            s += f"{var(name_jac, i=i, j=jf_first_[i])} = " \
                 f"{var(name_dict, i=0, j=pfi_[i])}{matmul}{var(name_jac, i=i, j=jf_first_[i])}{nl}"

        ijf_i = ijf[i, :]
        ijf_i[:jf_last[i] + 1] = False
        ijf_i = np.nonzero(ijf_i)[0]
        nf_i = nfi[jf_last[i]]

        # Handle joints which act on multiple frames
        if jf_first[i] != jf_last[i]:
            for k, fj_cur in enumerate(jf_all[i][:-1]):
                jf_next = jf_all[i][k + 1]
                jf_next_1 = jf_next - 1

                if jf_next - fj_cur > 1:
                    for j in range(fj_cur+1, jf_next):
                        for ll in range(fj_cur, jf_next-1):
                            s += f"{var(name_jac, i=i, j=j)} = " \
                                 f"{var(name_jac, i=i, j=fj_cur)}{matmul}{var(name_dict, i=nfi[fj_cur], j=ll+1)}{nl}"

                s += (f"{var(name_jac, i=i, j=jf_next)} = "
                      f"{var(name_jac, i=i, j=jf_next_1)}{matmul}{var(name_dict, i=nfi[jf_next_1], j=nfi[jf_next_1])} +"
                      f" {var(name_dict, i=0, j=jf_next_1)}{matmul}{var(name_jac, i=i, j=jf_next)}{nl}")

        # j(b)__a_c = j__a_b * f__b_c
        if isinstance(nf_i, (list, tuple)):
            nf_i_all = [nf_i_j for nf_i_j in nf_i for _ in range(iff[nf_i_j].sum())]
            for ijf_ij, nfi_ii in zip(ijf_i, nf_i_all):
                s += f"{var(name_jac, i=i, j=ijf_ij)} = " \
                     f"{var(name_jac, i=i, j=jf_last[i])}{matmul}{var(name_dict, i=nfi_ii, j=ijf_ij)}{nl}"

        elif nf_i != -1:
            for ijf_ij in ijf_i:
                s += f"{var(name_jac, i=i, j=ijf_ij)} = " \
                    f"{var(name_jac, i=i, j=jf_last[i])}{matmul}{var(name_dict, i=nf_i, j=ijf_ij)}{nl}"

    return s


def remove_trailing_zeros(s, ends=(',', ';', ')', ']', '+', '-', '*', '/', '\n'), n=10):
    for _ in range(n):
        for e in ends:
            s = s.replace(f'00{e}', e)
        if s[-2:] == '00':
            s = s[:-2]
    return s


def find_x123(x, s):
    pattern = rf"{x}\d{'{1,3}'}"    # get all x1 - x999
    pattern = re.compile(pattern)
    return [(match, int(match.group()[len(x):])) for match in pattern.finditer(s)]


def x2str(s, old, new):
    matches = find_x123(x=old, s=s)
    for m in reversed(matches):
        s = f"{s[:m[0].start()]}" \
            f"{new.format(i=m[1])}" \
            f"{s[m[0].end():]}"

    return s


def q2str(s):
    return x2str(s=s, old=name_joint, new=cpp_joint_i)


def dh2str(s):
    for j in range(5):
        s = x2str(s, old=name_dh5[j], new=cpp_dh_ij.format(i='{i}', j=j))
    return s


def math2str2(s, fun):
    pattern = rf"{fun}\({name_joint}\d{'{1,3}'}\)"    # get all q1 - q999
    p = re.compile(pattern)
    m = [m for m in p.finditer(s)]
    for mm in reversed(m):
        s = f"{s[:mm.start()]}" \
            f"{fun}" \
            f"{brackets_sis.format(i=mm.group()[len(fun) + 1 + len(name_joint):-1])}" \
            f"{s[mm.end():]}"

    return s


def math2str(s):
    for key in math_dict:
        s = s.replace(key, math_dict[key])
    return s


def test_q2str():
    s0 = "2 * q1 + q0 + q00 +3*cos(q20 + q15)"
    s00 = "2 * sin(q1) + q0 + q00 +3*cos(q20)"
    s1 = '2 * q(1) + q(0) + q(00) +3*cos(q(20) + q(15))'

    assert s1 == q2str(s0)
    assert s1 == math2str2(s=s00, fun='cos')


def f_astype_str(f):
    mat2 = np.zeros(f.shape, dtype=object)
    for i in range(f.shape[0]):
        for j in range(f.shape[1]):
            s = str(f[i, j])
            s = remove_trailing_zeros(s=s)
            s = math2str(s)
            # s = math2str(s)
            s = q2str(s)
            s = dh2str(s)
            mat2[i, j] = s

    return mat2


def frame2str_eigen(name, f):
    f = f_astype_str(f)
    s = f"{name} << "
    offset = len(s)

    for i in range(f.shape[0]):
        for j in range(f.shape[1]):
            s += f"{f[i, j]}, "

        if i == f.shape[0]-1:
            s = s[:-2] + ';\n'
        else:
            s = s[:-1] + '\n' + ' ' * offset

    return s


def frame2str_np(mat, mat_name):

    s = "\n{} = np.zeros(m_shape)".format(mat_name)
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            if mat[i, j] != 0:
                s += "\n{mat_name}[..., {i}, {j}] = {ele}".format(mat_name=mat_name, i=i, j=j, ele=str(mat[i, j]))
    s += '\n'
    return s


def mat_list2fun_string_np(mat_list, mat_name):
    fun_s = '\n'
    for i, torso_mat in enumerate(mat_list):
        fun_s += frame2str_np(mat=torso_mat, mat_name='{name}{i}'.format(name=mat_name, i=i))
    fun_s += '\n'
    return fun_s
