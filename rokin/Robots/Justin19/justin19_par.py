from numpy import pi, deg2rad, array, stack, vstack, zeros

from rokin.Robots.Justin19.spheres import SPHERES_F_IDX, SPHERES_POS, SPHERES_RAD  # noqa
from rokin.Robots.Justin19.capsules import CAPSULES_F_IDX, CAPSULES_POS, CAPSULES_RAD  # noqa
from rokin.Robots.Justin19.masses import MASSES, MASS_F_IDX, MASS_POS  # noqa
from rokin.Robots.Justin19.meshes import MESHES, MESHES_F_IDX, MESHES_F  # noqa

try:
    import ardx.ardx as ardx

    ardx_crobotkinematics = ardx.require('ajustin.planner.crobotkinematics-wrapper')
    IDX_F_TORSO_BASE = ardx.constants.AJUSTIN_KINEMATICS_FRAME_BASE
    IDX_F_TORSO_TCP = 4
    IDX_F_RIGHT_BASE = ardx.constants.AJUSTIN_KINEMATICS_FRAME_RIGHT_BASE
    IDX_F_RIGHT_TCP = ardx.constants.AJUSTIN_KINEMATICS_FRAME_RIGHT_TCP
    IDX_F_LEFT_BASE = ardx.constants.AJUSTIN_KINEMATICS_FRAME_LEFT_BASE
    IDX_F_LEFT_TCP = ardx.constants.AJUSTIN_KINEMATICS_FRAME_LEFT_TCP
    IDX_F_HEAD_BASE = ardx.constants.AJUSTIN_KINEMATICS_FRAME_HEAD_BASE
    IDX_F_HEAD_TCP = ardx.constants.AJUSTIN_KINEMATICS_FRAME_HEAD2

except ModuleNotFoundError:
    # print(f"{__name__}: ardx not found")
    ardx = None
    IDX_F_TORSO_BASE = 0
    IDX_F_TORSO_TCP = 4
    IDX_F_RIGHT_BASE = 5
    IDX_F_RIGHT_TCP = 13
    IDX_F_LEFT_BASE = 14
    IDX_F_LEFT_TCP = 22
    IDX_F_HEAD_BASE = 23
    IDX_F_HEAD_TCP = 26


# IDX_JOINTS
# Mapping of all DoF's       # Number of joints per body part
IDX_JOINTS = range(0, 19)
IDX_J_TORSO = range(0, 3)   # 3
IDX_J_RIGHT = range(3, 10)  # 7
IDX_J_LEFT = range(10, 17)  # 7
IDX_J_HEAD = range(17, 19)  # 2

N_JOINTS_TORSO = 3
N_JOINTS_RIGHT = 7
N_JOINTS_LEFT = 7
N_JOINTS_HEAD = 2
N_JOINTS = N_JOINTS_TORSO + N_JOINTS_RIGHT + N_JOINTS_LEFT + N_JOINTS_HEAD  # 19

# Labeling of the joints
IDX_J_TORSO_PAN = 0               # Rotates torso
IDX_J_TORSO_TILTING_1 = 1         # Coupled joints for the tilting of the torso
IDX_J_TORSO_TILTING_2 = 2         #

#                                 # See 'Vitruvian Man' (Da Vinci, 1490) for initial pose of Justin
IDX_J_RIGHT_SHOULDER_ROT = 3      # Rotates upper arm along axis
IDX_J_RIGHT_SHOULDER_UP_DOWN = 4  # Rotates upper arm up/down [flex] (up=+)
IDX_J_RIGHT_ELBOW_ROT = 5         # Rotates forearm along axis
IDX_J_RIGHT_ELBOW_LEFT_RIGHT = 6  # Rotates forearm left/right [flex] (left=+)
IDX_J_RIGHT_WRIST_ROT = 7         # Rotates wrist along axis
IDX_J_RIGHT_WRIST_RIGHT_LEFT = 8  # Rotates wrist [wigwag] (right=+)
IDX_J_RIGHT_WRIST_DOWN_UP = 9     # Rotates wrist around outstretched thumb [motor bike gas pedal] (down=+)

IDX_J_LEFT_SHOULDER_ROT = 10
IDX_J_LEFT_SHOULDER_UP_DOWN = 11
IDX_J_LEFT_ELBOW_ROT = 12
IDX_J_LEFT_ELBOW_LEFT_RIGHT = 13
IDX_J_LEFT_WRIST_ROT = 14
IDX_J_LEFT_WRIST_RIGHT_LEFT = 15
IDX_J_LEFT_WRIST_DOWN_UP = 16

IDX_J_HEAD_PAN = 17
IDX_J_HEAD_TILT = 18

IDX_J_COUPLED_TORSO_TILTING = [IDX_J_TORSO_TILTING_1, IDX_J_TORSO_TILTING_2]

# IDX_FRAMES
# Mapping of all frames                                     # Number of frames per body part
IDX_F_TORSO = range(IDX_F_TORSO_BASE, IDX_F_TORSO_TCP + 1)  # 5
IDX_F_RIGHT = range(IDX_F_RIGHT_BASE, IDX_F_RIGHT_TCP + 1)  # 9
IDX_F_LEFT = range(IDX_F_LEFT_BASE, IDX_F_LEFT_TCP + 1)     # 9
IDX_F_HEAD = range(IDX_F_HEAD_BASE, IDX_F_HEAD_TCP + 1)     # 4
IDX_FRAMES = range(IDX_F_TORSO_BASE, IDX_F_HEAD_TCP + 1)    # 27
N_FRAMES_TORSO = len(IDX_F_TORSO)
N_FRAMES_RIGHT = len(IDX_F_RIGHT)
N_FRAMES_LEFT = len(IDX_F_LEFT)
N_FRAMES_HEAD = len(IDX_F_HEAD)
N_FRAMES = len(IDX_FRAMES)


# DH Parameter
DH_TORSO = array([[0.1055, 0, 0, 0],
                  [0, -pi / 2, 0, -pi / 2],
                  [0, 0, 0.3, 0],
                  [0, 0, 0.3, 0]])
DH_RIGHT = array([[0, 0, 0, 0],
                  [0, 0, 0, pi / 2],
                  [0.4, -pi / 2, 0, -pi / 2],
                  [0, 0, 0, pi / 2],
                  [0.39, -pi, 0, -pi / 2],
                  [0, pi / 2, 0, pi / 2],
                  [0, -pi / 2, 0, pi / 2]])
DH_LEFT = array([[0, 0, 0, pi],
                 [0, 0, 0, pi / 2],
                 [-0.4, -pi / 2, 0, -pi / 2],
                 [0, 0, 0, pi / 2],
                 [-0.39, 0, 0, -pi / 2],
                 [0, pi / 2, 0, -pi / 2],
                 [0, -pi / 2, 0, -pi / 2]])
DH_HEAD = array([[0, 0, 0, 0],
                 [0, 0, 0, -pi / 2]])
DH = vstack((DH_TORSO, DH_RIGHT, DH_LEFT, DH_HEAD))

# f_mobileBase_torsoBase
F_TORSO_BASE = array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.5884],  # measured from base plate which is approx. 10cm above the floor
                      [0, 0, 0, 1]])
# f_torsoLast_rightBase
F_RIGHT_BASE = array([[-0.866, 0, 0.5, 0.190],
                      [0, 1, 0, 0.088],
                      [-0.5, 0, -0.866, -0.256],
                      [0, 0, 0, 1]])
F_LEFT_BASE = array([[0.866, 0, 0.5, 0.190],
                     [0, 1, 0, 0.088],
                     [-0.5, 0, 0.866, 0.256],
                     [0, 0, 0, 1]])
F_HEAD_BASE = array([[0, 0, 1, 0.235],
                     [1, 0, 0, 0.088],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]])
F_RIGHT_TCP = array([[-1, 0, 0, 0],
                     [0, 0, 1, 0.118],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]])
F_LEFT_TCP = F_RIGHT_TCP
F_HEAD_TCP = array([[1, 0, 0, 0],
                    [0, 0, -1, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
F_STATIC = stack((F_TORSO_BASE,
                  F_RIGHT_BASE, F_RIGHT_TCP,
                  F_LEFT_BASE, F_LEFT_TCP,
                  F_HEAD_BASE, F_HEAD_TCP), axis=0)

IDX_F_STATIC = array([IDX_F_TORSO_BASE,
                      IDX_F_RIGHT_BASE, IDX_F_RIGHT_TCP,
                      IDX_F_LEFT_BASE, IDX_F_LEFT_TCP,
                      IDX_F_HEAD_BASE, IDX_F_HEAD_TCP])

# ForwardKinematic Chain
next_frame_idx = array([1, 2, 3, 4, [5, 14, 23],             # 0-4
                        6, 7, 8, 9, 10, 11, 12, 13, -1,      # 5-13
                        15, 16, 17, 18, 19, 20, 21, 22, -1,  # 14-22
                        24, 25, 26, -1], dtype=object)       # 22-26

joint_frame_idx = array([1, [2, 4], [3, 4],           # 0-2
                         6, 7, 8, 9, 10, 11, 12,      # 3-10
                         15, 16, 17, 18, 19, 20, 21,  # 11-17
                         24, 25], dtype=object)       # 18-19

joint_frame_idx_dh = array([1, 2, 3, 4,                  # 0-3
                            6, 7, 8, 9, 10, 11, 12,      # 4-11
                            15, 16, 17, 18, 19, 20, 21,  # 12-18
                            24, 25])                     # 19-20

temp = zeros(19)
temp[[1, 2]] = -1
coupled_passive_joints = {3: lambda q: -q[1] - q[2]}
coupled_passive_joints_jac = {3: lambda q: temp}

FRAME_UPPER_BODY_COM = 4

# JOINT CONSTRAINTS
# from /home/baeuml/src/casadi-planner/planner/ajustin-collision.rkt

# joint3 handled with additional linear constraint
# if joint2 < 0:
#     joint3 = [0-joint2, 135       ]
# else:
#     joint3 = [0       , 135-joint2]


# torso0 was [-140, 200]
#   but this hurts the cables and the controllers are not able to fulfil those extreme angles
#   -> make it more conservative = symmetrical
JOINT_LIMITS_TORSO = deg2rad([[-120., 120.],
                              [-90., 90.],
                              [0., 135.]])
JOINT_LIMITS_RIGHT = deg2rad([[-170., 170.],
                              [-120., 120.],
                              [-170., 170.],
                              [-120., 120.],
                              [-170., 170.],
                              [-45.1, 80.],
                              [-45., 135.]])
JOINT_LIMITS_LEFT = deg2rad([[-170., 170.],
                             [-120., 120.],
                             [-170., 170.],
                             [-120., 120.],
                             [-170., 170.],
                             [-45.1, 80.],
                             [-45., 135.]])
JOINT_LIMITS_HEAD = deg2rad([[-45.2, 45.2],
                             [-22.5, 45.]])

JOINT_LIMITS = vstack((JOINT_LIMITS_TORSO,
                       JOINT_LIMITS_RIGHT,
                       JOINT_LIMITS_LEFT,
                       JOINT_LIMITS_HEAD))

# Add safety factor
JOINT_LIMITS_SAFETY_FACTOR = deg2rad(3)

JOINT_LIMITS_TORSO2 = JOINT_LIMITS_TORSO + JOINT_LIMITS_SAFETY_FACTOR * array([1, -1])
JOINT_LIMITS_RIGHT2 = JOINT_LIMITS_RIGHT + JOINT_LIMITS_SAFETY_FACTOR * array([1, -1])
JOINT_LIMITS_LEFT2 = JOINT_LIMITS_LEFT + JOINT_LIMITS_SAFETY_FACTOR * array([1, -1])
JOINT_LIMITS_HEAD2 = JOINT_LIMITS_HEAD + JOINT_LIMITS_SAFETY_FACTOR * array([1, -1])

JOINT_LIMITS2 = vstack((JOINT_LIMITS_TORSO2,
                        JOINT_LIMITS_RIGHT2,
                        JOINT_LIMITS_LEFT2,
                        JOINT_LIMITS_HEAD2))


MAX_JOINT_VELOCITY = deg2rad(100)  # max 100 grad pro s
MAX_JOINT_ACCELERATION = MAX_JOINT_VELOCITY / 2  # max 0.5s to reach vq_max


# # Length of the limbs of Justin's arms in [matrix]  -> total arm length: 0.908 matrix
#                        upper arm  forearm  hand
# ARM_LENGTHS = np.array([0.400,    0.390,   0.118])

def get_frame_name(f_idx):
    if f_idx in IDX_F_TORSO:
        return 'Torso {}'.format(f_idx - IDX_F_TORSO.start)
    elif f_idx in IDX_F_RIGHT:
        return 'Arm Right {}'.format(f_idx - IDX_F_RIGHT.start)
    elif f_idx in IDX_F_LEFT:
        return 'Arm Left {}'.format(f_idx - IDX_F_LEFT.start)
    elif f_idx in IDX_F_HEAD:
        return 'Head {}'.format(f_idx - IDX_F_HEAD.start)
    else:
        raise AttributeError('Frame idx{} not known'.format(f_idx))


def q_justin_body_parts(q):
    q_torso = array([q[..., 0], q[..., 1], q[..., 2], -q[..., 1] - q[..., 2]]).T
    q_right = q[..., 3:10]
    q_left = q[..., 10:17]
    q_head = q[..., 17:19]

    return q_torso, q_right, q_left, q_head
