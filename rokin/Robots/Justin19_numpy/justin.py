import numpy as np

from rokin.Robots.Justin19 import justin19_par as jtp
from wzk import slice_add, range2slice

IDX_F_TORSO_WO_BASE = slice(jtp.IDX_F_TORSO.start + 1, jtp.IDX_F_TORSO.stop, 1)  # 4
IDX_F_RIGHT_WO_BASE = slice(jtp.IDX_F_RIGHT.start + 1, jtp.IDX_F_RIGHT.stop, 1)  # 8
IDX_F_LEFT_WO_BASE = slice(jtp.IDX_F_LEFT.start + 1, jtp.IDX_F_LEFT.stop, 1)     # 8
IDX_F_HEAD_WO_BASE = slice(jtp.IDX_F_HEAD.start + 1, jtp.IDX_F_HEAD.stop, 1)     # 3

FRAMES_TORSO_WO_BASE_SLICE0 = slice_add(IDX_F_TORSO_WO_BASE, (-1, -1, 0))
FRAMES_RIGHT_WO_BASE_SLICE0 = slice_add(IDX_F_RIGHT_WO_BASE, (-1, -1, 0))
FRAMES_LEFT_WO_BASE_SLICE0 = FRAMES_RIGHT_WO_BASE_SLICE0
FRAMES_HEAD_WO_BASE_SLICE0 = slice(5, 8, 1)

IDX_F_RIGHT_BASE0 = jtp.IDX_F_RIGHT_BASE - 1
IDX_F_LEFT_BASE0 = IDX_F_RIGHT_BASE0
IDX_F_HEAD_BASE0 = IDX_F_RIGHT_BASE0

IDX_J_TORSO = range2slice(jtp.IDX_J_TORSO)
IDX_J_RIGHT = range2slice(jtp.IDX_J_RIGHT)
IDX_J_LEFT = range2slice(jtp.IDX_J_LEFT)
IDX_J_HEAD = range2slice(jtp.IDX_J_HEAD)
IDX_JOINTS = range2slice(jtp.IDX_JOINTS)

IDX_J_TORSO0 = range2slice(jtp.IDX_J_TORSO)
IDX_J_RIGHT0 = range2slice(jtp.IDX_J_RIGHT)
IDX_J_LEFT0 = IDX_J_RIGHT0
IDX_J_HEAD0 = slice(3, 5, 1)

#  Base, justin_cython  -  Base, Right  -  Base, Left  -  Base, Head
#  justin_cython  -  Base, Right
#  justin_cython  -  Base, Left
#  justin_cython  -  Base, Head
#  Right
#  Left

# Finding you do not need constant base frames for the jacobian


def __get_sin_cos_justin(q):
    sin_q = np.sin(q)
    cos_q = np.cos(q)
    sin_q12 = np.sin(q[..., jtp.IDX_J_TORSO_TILTING_1] + q[..., jtp.IDX_J_TORSO_TILTING_2])
    cos_q12 = np.cos(q[..., jtp.IDX_J_TORSO_TILTING_1] + q[..., jtp.IDX_J_TORSO_TILTING_2])
    return sin_q, cos_q, sin_q12, cos_q12


# Fill frames modular
def __torso_base(frames):
    frames[..., 0, 0] = 1.
    frames[..., 1, 1] = 1.
    frames[..., 2, 2] = 1.
    frames[..., 2, 3] = 0.58840


def __torso(frames, sc):
    """
    In Justin:
    frames start at 1
    joints start at 0
    """

    sin_q, cos_q, sin_q12, cos_q12 = sc

    # torso0
    frames[..., 0, 0, 0] = cos_q[..., 0]
    frames[..., 0, 0, 1] = -sin_q[..., 0]
    frames[..., 0, 1, 0] = sin_q[..., 0]
    frames[..., 0, 1, 1] = cos_q[..., 0]
    frames[..., 0, 2, 2] = 1
    frames[..., 0, 2, 3] = 0.1055

    # torso1
    frames[..., 1, 0, 0] = sin_q[..., 1]
    frames[..., 1, 0, 1] = cos_q[..., 1]
    frames[..., 1, 1, 2] = 1
    frames[..., 1, 2, 0] = cos_q[..., 1]
    frames[..., 1, 2, 1] = -sin_q[..., 1]

    # torso2
    frames[..., 2, 0, 0] = cos_q[..., 2]
    frames[..., 2, 0, 1] = -sin_q[..., 2]
    frames[..., 2, 1, 0] = sin_q[..., 2]
    frames[..., 2, 1, 1] = cos_q[..., 2]
    frames[..., 2, 2, 2] = 1
    frames[..., 2, 0, 3] = 0.3

    # torso3
    frames[..., 3, 0, 0] = cos_q12
    frames[..., 3, 0, 1] = sin_q12
    frames[..., 3, 1, 0] = -sin_q12
    frames[..., 3, 1, 1] = cos_q12
    frames[..., 3, 2, 2] = 1
    frames[..., 3, 0, 3] = 0.3


def __torso_jac(frames_jac, sc):

    sin_q, cos_q, sin_q12, cos_q12 = sc

    frames_jac[..., 0, 0, 0, 0] = -sin_q[..., 0]
    frames_jac[..., 0, 0, 0, 1] = -cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 0] = cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 1] = -sin_q[..., 0]

    frames_jac[..., 1, 1, 0, 0] = cos_q[..., 1]
    frames_jac[..., 1, 1, 0, 1] = -sin_q[..., 1]
    frames_jac[..., 1, 1, 2, 0] = -sin_q[..., 1]
    frames_jac[..., 1, 1, 2, 1] = -cos_q[..., 1]

    frames_jac[..., 2, 2, 0, 0] = -sin_q[..., 2]
    frames_jac[..., 2, 2, 0, 1] = -cos_q[..., 2]
    frames_jac[..., 2, 2, 1, 0] = cos_q[..., 2]
    frames_jac[..., 2, 2, 1, 1] = -sin_q[..., 2]

    frames_jac[..., 1, 3, 0, 0] = -sin_q12
    frames_jac[..., 1, 3, 0, 1] = cos_q12
    frames_jac[..., 1, 3, 1, 0] = -cos_q12
    frames_jac[..., 1, 3, 1, 1] = -sin_q12

    frames_jac[..., 2, 3, 0, 0] = -sin_q12
    frames_jac[..., 2, 3, 0, 1] = cos_q12
    frames_jac[..., 2, 3, 1, 0] = -cos_q12
    frames_jac[..., 2, 3, 1, 1] = -sin_q12


def __right_base(frames):
    frames[..., 0, 0] = -0.866
    frames[..., 0, 2] = 0.5
    frames[..., 1, 1] = 1.
    frames[..., 2, 0] = -0.5
    frames[..., 2, 2] = -0.866
    frames[..., 0, 3] = 0.190
    frames[..., 1, 3] = 0.0880
    frames[..., 2, 3] = -0.256


def __right(frames, sc):
    """
    In Justin:
    frames start at 6
    joints start at 3
    """

    sin_q, cos_q = sc

    # right 0
    frames[..., 0, 0, 0] = cos_q[..., 0]
    frames[..., 0, 0, 1] = -sin_q[..., 0]
    frames[..., 0, 1, 0] = sin_q[..., 0]
    frames[..., 0, 1, 1] = cos_q[..., 0]
    frames[..., 0, 2, 2] = 1

    # right 1
    frames[..., 1, 0, 0] = cos_q[..., 1]
    frames[..., 1, 0, 1] = -sin_q[..., 1]
    frames[..., 1, 1, 2] = -1
    frames[..., 1, 2, 0] = sin_q[..., 1]
    frames[..., 1, 2, 1] = cos_q[..., 1]

    # right 2
    frames[..., 2, 0, 0] = sin_q[..., 2]
    frames[..., 2, 0, 1] = cos_q[..., 2]
    frames[..., 2, 1, 2] = 1
    frames[..., 2, 2, 0] = cos_q[..., 2]
    frames[..., 2, 2, 1] = -sin_q[..., 2]
    frames[..., 2, 1, 3] = 0.4

    # right 3
    frames[..., 3, 0, 0] = cos_q[..., 3]
    frames[..., 3, 0, 1] = -sin_q[..., 3]
    frames[..., 3, 1, 2] = -1
    frames[..., 3, 2, 0] = sin_q[..., 3]
    frames[..., 3, 2, 1] = cos_q[..., 3]

    # right 4
    frames[..., 4, 0, 0] = -cos_q[..., 4]
    frames[..., 4, 0, 1] = sin_q[..., 4]
    frames[..., 4, 1, 2] = 1
    frames[..., 4, 2, 0] = sin_q[..., 4]
    frames[..., 4, 2, 1] = cos_q[..., 4]
    frames[..., 4, 1, 3] = 0.390

    # right 5
    frames[..., 5, 0, 0] = -sin_q[..., 5]
    frames[..., 5, 0, 1] = -cos_q[..., 5]
    frames[..., 5, 1, 2] = -1
    frames[..., 5, 2, 0] = cos_q[..., 5]
    frames[..., 5, 2, 1] = -sin_q[..., 5]

    # right 6
    frames[..., 6, 0, 0] = sin_q[..., 6]
    frames[..., 6, 0, 1] = cos_q[..., 6]
    frames[..., 6, 1, 2] = -1
    frames[..., 6, 2, 0] = -cos_q[..., 6]
    frames[..., 6, 2, 1] = sin_q[..., 6]

    # right 7  # TODO extract head to separate function
    frames[..., 7, 0, 0] = -1
    frames[..., 7, 1, 2] = 1
    frames[..., 7, 2, 1] = 1
    frames[..., 7, 1, 3] = 0.118


def __right_jac(frames_jac, sc):
    """
    In Justin:
    frames start at 6
    joints start at 3
    """

    sin_q, cos_q = sc

    # right 0
    frames_jac[..., 0, 0, 0, 0] = -sin_q[..., 0]
    frames_jac[..., 0, 0, 0, 1] = -cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 0] = cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 1] = -sin_q[..., 0]

    # right 1
    frames_jac[..., 1, 1, 0, 0] = -sin_q[..., 1]
    frames_jac[..., 1, 1, 0, 1] = -cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 0] = cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 1] = -sin_q[..., 1]

    # right 2
    frames_jac[..., 2, 2, 0, 0] = cos_q[..., 2]
    frames_jac[..., 2, 2, 0, 1] = -sin_q[..., 2]
    frames_jac[..., 2, 2, 2, 0] = -sin_q[..., 2]
    frames_jac[..., 2, 2, 2, 1] = -cos_q[..., 2]

    # right 3
    frames_jac[..., 3, 3, 0, 0] = -sin_q[..., 3]
    frames_jac[..., 3, 3, 0, 1] = -cos_q[..., 3]
    frames_jac[..., 3, 3, 2, 0] = cos_q[..., 3]
    frames_jac[..., 3, 3, 2, 1] = -sin_q[..., 3]

    # right 4
    frames_jac[..., 4, 4, 0, 0] = sin_q[..., 4]
    frames_jac[..., 4, 4, 0, 1] = cos_q[..., 4]
    frames_jac[..., 4, 4, 2, 0] = cos_q[..., 4]
    frames_jac[..., 4, 4, 2, 1] = -sin_q[..., 4]

    # right 5
    frames_jac[..., 5, 5, 0, 0] = -cos_q[..., 5]
    frames_jac[..., 5, 5, 0, 1] = sin_q[..., 5]
    frames_jac[..., 5, 5, 2, 0] = -sin_q[..., 5]
    frames_jac[..., 5, 5, 2, 1] = -cos_q[..., 5]

    # right 6
    frames_jac[..., 6, 6, 0, 0] = cos_q[..., 6]
    frames_jac[..., 6, 6, 0, 1] = -sin_q[..., 6]
    frames_jac[..., 6, 6, 2, 0] = sin_q[..., 6]
    frames_jac[..., 6, 6, 2, 1] = cos_q[..., 6]

    # right 7


def __left_base(frames):
    frames[..., :-1, :] = np.array([[0.866,   0,   0.5,  0.19],
                                    [0,       1,     0,  0.088],
                                    [-0.5,    0, 0.866, 0.256]])


def __left(frames, sc):
    """
    In Justin:
    frames start at 15
    joints start at 10
    """

    sin_q, cos_q = sc

    # left_arm0
    frames[..., 0, 0, 0] = cos_q[..., 0]
    frames[..., 0, 0, 1] = -sin_q[..., 0]
    frames[..., 0, 1, 0] = -sin_q[..., 0]
    frames[..., 0, 1, 1] = -cos_q[..., 0]
    frames[..., 0, 2, 2] = -1

    # left_arm1
    frames[..., 1, 0, 0] = cos_q[..., 1]
    frames[..., 1, 0, 1] = -sin_q[..., 1]
    frames[..., 1, 1, 2] = -1
    frames[..., 1, 2, 0] = sin_q[..., 1]
    frames[..., 1, 2, 1] = cos_q[..., 1]

    # left_arm2
    frames[..., 2, 0, 0] = sin_q[..., 2]
    frames[..., 2, 0, 1] = cos_q[..., 2]
    frames[..., 2, 1, 2] = 1
    frames[..., 2, 1, 3] = -0.4
    frames[..., 2, 2, 0] = cos_q[..., 2]
    frames[..., 2, 2, 1] = -sin_q[..., 2]

    # left_arm3
    frames[..., 3, 0, 0] = cos_q[..., 3]
    frames[..., 3, 0, 1] = -sin_q[..., 3]
    frames[..., 3, 1, 2] = -1
    frames[..., 3, 2, 0] = sin_q[..., 3]
    frames[..., 3, 2, 1] = cos_q[..., 3]

    # left_arm4
    frames[..., 4, 0, 0] = cos_q[..., 4]
    frames[..., 4, 0, 1] = -sin_q[..., 4]
    frames[..., 4, 1, 2] = 1
    frames[..., 4, 1, 3] = -0.390
    frames[..., 4, 2, 0] = -sin_q[..., 4]
    frames[..., 4, 2, 1] = -cos_q[..., 4]

    # left_arm5
    frames[..., 5, 0, 0] = -sin_q[..., 5]
    frames[..., 5, 0, 1] = -cos_q[..., 5]
    frames[..., 5, 1, 2] = 1
    frames[..., 5, 2, 0] = -cos_q[..., 5]
    frames[..., 5, 2, 1] = sin_q[..., 5]

    # left_arm6
    frames[..., 6, 0, 0] = sin_q[..., 6]
    frames[..., 6, 0, 1] = cos_q[..., 6]
    frames[..., 6, 1, 2] = 1
    frames[..., 6, 2, 0] = cos_q[..., 6]
    frames[..., 6, 2, 1] = -sin_q[..., 6]

    # left_arm7 TODO extract
    frames[..., 7, 0, 0] = -1
    frames[..., 7, 1, 2] = 1
    frames[..., 7, 1, 3] = 0.118
    frames[..., 7, 2, 1] = 1


def __left_jac(frames_jac, sc):
    """
    In Justin:
    frames start at 15
    joints start at 10
    """

    sin_q, cos_q = sc

    # left 0
    frames_jac[..., 0, 0, 0, 0] = -sin_q[..., 0]
    frames_jac[..., 0, 0, 0, 1] = -cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 0] = -cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 1] = sin_q[..., 0]

    # left 1
    frames_jac[..., 1, 1, 0, 0] = -sin_q[..., 1]
    frames_jac[..., 1, 1, 0, 1] = -cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 0] = cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 1] = -sin_q[..., 1]

    # left 2
    frames_jac[..., 2, 2, 0, 0] = cos_q[..., 2]
    frames_jac[..., 2, 2, 0, 1] = -sin_q[..., 2]
    frames_jac[..., 2, 2, 2, 0] = -sin_q[..., 2]
    frames_jac[..., 2, 2, 2, 1] = -cos_q[..., 2]

    # left 3
    frames_jac[..., 3, 3, 0, 0] = -sin_q[..., 3]
    frames_jac[..., 3, 3, 0, 1] = -cos_q[..., 3]
    frames_jac[..., 3, 3, 2, 0] = cos_q[..., 3]
    frames_jac[..., 3, 3, 2, 1] = -sin_q[..., 3]

    # left 4
    frames_jac[..., 4, 4, 0, 0] = -sin_q[..., 4]
    frames_jac[..., 4, 4, 0, 1] = -cos_q[..., 4]
    frames_jac[..., 4, 4, 2, 0] = -cos_q[..., 4]
    frames_jac[..., 4, 4, 2, 1] = sin_q[..., 4]

    # left 5
    frames_jac[..., 5, 5, 0, 0] = -cos_q[..., 5]
    frames_jac[..., 5, 5, 0, 1] = sin_q[..., 5]
    frames_jac[..., 5, 5, 2, 0] = sin_q[..., 5]
    frames_jac[..., 5, 5, 2, 1] = cos_q[..., 5]

    # left 6
    frames_jac[..., 6, 6, 0, 0] = cos_q[..., 6]
    frames_jac[..., 6, 6, 0, 1] = -sin_q[..., 6]
    frames_jac[..., 6, 6, 2, 0] = -sin_q[..., 6]
    frames_jac[..., 6, 6, 2, 1] = -cos_q[..., 6]


def __head_base(frames):
    frames[..., 0, 2] = 1.
    frames[..., 0, 3] = 0.235
    frames[..., 1, 0] = 1.
    frames[..., 1, 3] = 0.088
    frames[..., 2, 1] = 1.


def __head(frames, sc):
    """
    In Justin:
    frames start at 24
    joints start at 17
    """

    sin_q, cos_q = sc

    # head0
    frames[..., 0, 0, 0] = cos_q[..., 0]
    frames[..., 0, 0, 1] = -sin_q[..., 0]
    frames[..., 0, 1, 0] = sin_q[..., 0]
    frames[..., 0, 1, 1] = cos_q[..., 0]
    frames[..., 0, 2, 2] = 1

    # head1
    frames[..., 1, 0, 0] = cos_q[..., 1]
    frames[..., 1, 0, 1] = -sin_q[..., 1]
    frames[..., 1, 1, 2] = 1
    frames[..., 1, 2, 0] = -sin_q[..., 1]
    frames[..., 1, 2, 1] = -cos_q[..., 1]

    # head2  # TODO extract, not part of the fun
    frames[..., 2, 0, 0] = 1
    frames[..., 2, 1, 2] = -1
    frames[..., 2, 2, 1] = 1


def __head_jac(frames_jac, sc):
    """
    In Justin:
    frames start at 24
    joints start at 17
    """

    sin_q, cos_q = sc

    # head 0
    frames_jac[..., 0, 0, 0, 0] = -sin_q[..., 0]
    frames_jac[..., 0, 0, 0, 1] = -cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 0] = cos_q[..., 0]
    frames_jac[..., 0, 0, 1, 1] = -sin_q[..., 0]

    # head 1
    frames_jac[..., 1, 1, 0, 0] = -sin_q[..., 1]
    frames_jac[..., 1, 1, 0, 1] = -cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 0] = -cos_q[..., 1]
    frames_jac[..., 1, 1, 2, 1] = sin_q[..., 1]

    # head 2


# Fill frames body parts
def fill_frames_justin(frames, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso_base(frames[:, :, jtp.IDX_F_TORSO_BASE, :, :])
    __torso(frames=frames[:, :, IDX_F_TORSO_WO_BASE, :, :],
            sc=(sin_q[..., IDX_J_TORSO], cos_q[..., IDX_J_TORSO], sin_q12, cos_q12))

    __right_base(frames[:, :, jtp.IDX_F_RIGHT_BASE, :, :])
    __right(frames=frames[:, :, IDX_F_RIGHT_WO_BASE, :, :],
            sc=(sin_q[..., IDX_J_RIGHT], cos_q[..., IDX_J_RIGHT]))

    __left_base(frames[:, :, jtp.IDX_F_LEFT_BASE, :, :])
    __left(frames=frames[:, :, IDX_F_LEFT_WO_BASE, :],
           sc=(sin_q[..., IDX_J_LEFT], cos_q[..., IDX_J_LEFT]))

    __head_base(frames[:, :, jtp.IDX_F_HEAD_BASE, :, :])
    __head(frames=frames[:, :, IDX_F_HEAD_WO_BASE, :],
           sc=(sin_q[..., IDX_J_HEAD], cos_q[..., IDX_J_HEAD]))


def fill_frames_torso_right(frames, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso(frames=frames[:, :, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
            sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0], sin_q12, cos_q12))

    __right_base(frames[:, :, IDX_F_RIGHT_BASE0, :, :])
    __right(frames=frames[:, :, FRAMES_RIGHT_WO_BASE_SLICE0, :, :],
            sc=(sin_q[..., IDX_J_RIGHT0], cos_q[..., IDX_J_RIGHT0]))


def fill_frames_torso_left(frames, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso(frames=frames[:, :, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
            sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0],
                sin_q12, cos_q12))

    __left_base(frames[:, :, IDX_F_LEFT_BASE0, :, :])
    __left(frames=frames[:, :, FRAMES_LEFT_WO_BASE_SLICE0, :, :],
           sc=(sin_q[..., IDX_J_LEFT0], cos_q[..., IDX_J_LEFT0]))


def fill_frames_torso_head(frames, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso(frames=frames[:, :, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
            sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0],
                sin_q12, cos_q12))

    __head_base(frames[:, :, IDX_F_HEAD_BASE0, :, :])
    __head(frames=frames[:, :, FRAMES_HEAD_WO_BASE_SLICE0, :, :],
           sc=(sin_q[..., IDX_J_HEAD0], cos_q[..., IDX_J_HEAD0]))


def fill_frames_torso(frames, sc):
    __torso(frames=frames, sc=sc)


def fill_frames_right(frames, sc):
    __right(frames=frames, sc=sc)


def fill_frames_left(frames, sc):
    __left(frames=frames, sc=sc)


# Jacobian
def fill_frames_jac_justin(frames_jac, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    # torso
    __torso_jac(frames_jac=frames_jac[:, :, IDX_J_TORSO, IDX_F_TORSO_WO_BASE, :, :],
                sc=(sin_q[..., IDX_J_TORSO], cos_q[..., IDX_J_TORSO],
                    sin_q12, cos_q12))

    # right
    __right_jac(frames_jac=frames_jac[:, :, IDX_J_RIGHT, IDX_F_RIGHT_WO_BASE, :, :],
                sc=(sin_q[..., IDX_J_RIGHT], cos_q[..., IDX_J_RIGHT]))

    # left
    __left_jac(frames_jac=frames_jac[:, :, IDX_J_LEFT, IDX_F_LEFT_WO_BASE, :, :],
               sc=(sin_q[..., IDX_J_LEFT], cos_q[..., IDX_J_LEFT]))

    # head
    __head_jac(frames_jac=frames_jac[:, :, IDX_J_HEAD, IDX_F_HEAD_WO_BASE, :],
               sc=(sin_q[..., IDX_J_HEAD], cos_q[..., IDX_J_HEAD]))


def fill_frames_jac_torso_right(frames_jac, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso_jac(frames_jac=frames_jac[:, :, IDX_J_TORSO0, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
                sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0],
                    sin_q12, cos_q12))

    __right_jac(frames_jac=frames_jac[:, :, IDX_J_RIGHT0, FRAMES_RIGHT_WO_BASE_SLICE0, :, :],
                sc=(sin_q[..., IDX_J_RIGHT0], cos_q[..., IDX_J_RIGHT0]))


def fill_frames_jac_torso_left(frames_jac, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso_jac(frames_jac=frames_jac[:, :, IDX_J_TORSO0, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
                sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0],
                    sin_q12, cos_q12))

    __left_jac(frames_jac=frames_jac[:, :, IDX_J_LEFT0, FRAMES_LEFT_WO_BASE_SLICE0, :, :],
               sc=(sin_q[..., IDX_J_LEFT0], cos_q[..., IDX_J_LEFT0]))


def fill_frames_jac_torso_head(frames_jac, sc):
    sin_q, cos_q, sin_q12, cos_q12 = sc

    __torso_jac(frames_jac=frames_jac[:, :, IDX_J_TORSO0, FRAMES_TORSO_WO_BASE_SLICE0, :, :],
                sc=(sin_q[..., IDX_J_TORSO0], cos_q[..., IDX_J_TORSO0],
                    sin_q12, cos_q12))

    __head_jac(frames_jac=frames_jac[:, :, IDX_J_HEAD0, FRAMES_HEAD_WO_BASE_SLICE0, :, :],
               sc=(sin_q[..., IDX_J_HEAD0], cos_q[..., IDX_J_HEAD0]))


def fill_frames_jac_torso(frames_jac, sc):
    __torso_jac(frames_jac=frames_jac, sc=sc)


def fill_frames_jac_right(frames_jac, sc):
    __right_jac(frames_jac=frames_jac, sc=sc)


def fill_frames_jac_left(frames_jac, sc):
    __right_jac(frames_jac=frames_jac, sc=sc)


def speed_test():
    from wzk import tic, toc, new_fig
    n_test = np.logspace(start=0, stop=5, num=100).astype(int)

    m = 1000
    time_element = []
    time_array = []
    for n in n_test:
        a = np.zeros((n, 4, 4))
        tic()
        for i in range(m):
            a[..., 0, 0] = 0.0
            a[..., 1, 1] = 1.1
            a[..., 2, 2] = 2.2
            a[..., 2, 3] = 2.3
            # a[..., 3, 2] = 3.2
            # a[..., 0, 1] = 0.3
            # a[..., 0, 2] = 0.3
            # a[..., 0, 3] = 0.3
        time_element.append(toc(name=''))

        tic()
        for i in range(m):
            # a[..., :, :] = np.array([[0.0, 0, 0, 0],
            #                          [0, 1.1, 0, 0],
            #                          [0, 0, 2.2, 2.3],
            #                          [0, 0, 3.2, 3.3]])
            a[..., :, :] = np.array([[0.0, 0.1, 0.2, 0.3],
                                     [0, 1.1, 0, 0],
                                     [0, 0, 2.2, 2.3],
                                     [0, 0, 3.2, 3.3]])
        time_array.append(toc(name=''))

    # Finding for 8+ elements it is faster to use an array to assign all at once
    #  for an array sizes 500-1000+ (x4x4) it is faster to use an array
    #  -> unsatisfying middle ground
    # TODO test on institute pc / different cpus
    fig, ax = new_fig()
    time_factor = np.array(time_element) / np.array(time_array)
    ax.semilogx(n_test, time_factor, label='element / array')
    ax.hlines(y=1, xmin=0, xmax=max(n_test))
    ax.legend()
    print(np.stack([n_test, time_factor]).T)


if __name__ == '__main__':
    speed_test()
