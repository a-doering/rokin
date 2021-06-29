import numpy as np

# Sphere model of the mobile base
# 6 Spheres
# __r = 0.3
# __x = 0.3
# __y = 0.2
# BASE_SPHERES = np.array(((+__x, +__y, __r),
#                          (+__x, -__y, __r),
#                          (-__x, +__y, __r),
#                          (-__x, -__y, __r),
#                          (0, +__y, __r),
#                          (0, -__y, __r)))

# BASE_SPHERES = np.array(((0, 0, 3*__r),
#                          (0, 0, 3*__r),
#                          (0, 0, 3*__r),
#                          (0, 0, 3*__r)))

n_dim = 2

try:
    import ardx.ardx as ardx

    mpc = ardx.require('ajconfig.mobile-platform-calibration')

    WHEEL_MOUNTING_POS = np.zeros((4, 2))
    WHEEL_MOUNTING_POS[0, :] = (mpc.MPC_WHEEL_FL_X, mpc.MPC_WHEEL_FL_Y)
    WHEEL_MOUNTING_POS[1, :] = (mpc.MPC_WHEEL_FR_X, mpc.MPC_WHEEL_FR_Y)
    WHEEL_MOUNTING_POS[2, :] = (mpc.MPC_WHEEL_BR_X, mpc.MPC_WHEEL_BR_Y)
    WHEEL_MOUNTING_POS[3, :] = (mpc.MPC_WHEEL_BL_X, mpc.MPC_WHEEL_BL_Y)

except (ModuleNotFoundError,) as e:
    print(f"{__name__}: ardx not found")
    # Mounting position of the wheels (clock-wise)
    WHEEL_MOUNTING_POS = np.array([[+0.376314, +0.281961],   # front left
                                   [+0.299227, -0.224202],   # front right
                                   [-0.386183, -0.289356],   # back right
                                   [-0.313059, +0.234566]])  # back left

WHEEL_MOUNTING_POS = np.array([[+0.3, +0.2],   # front left
                               [+0.3, -0.2],   # front right
                               [-0.3, -0.2],   # back right
                               [-0.3, +0.2]])  # back left

N_WHEELS = WHEEL_MOUNTING_POS.shape[0]
WHEEL_MOUNTING_ANGLES = np.arctan2(WHEEL_MOUNTING_POS[:, 1], WHEEL_MOUNTING_POS[:, 0])
WHEEL_MOUNTING_RADII = np.linalg.norm(WHEEL_MOUNTING_POS, axis=-1)

# Collision model
radius_torso_spheres = 0.3 + 0.03  # (same as for upper body)
radius_wheel_spheres = 0.12 + 0.03  # 0.12 cm is True

SPHERES = np.array(((+0.1, 0, radius_torso_spheres),  # justin_cython (same as for upper body)
                    (-0.1, 0, radius_torso_spheres),
                    (0, 0, radius_wheel_spheres),  # FourWheels
                    (0, 0, radius_wheel_spheres),
                    (0, 0, radius_wheel_spheres),
                    (0, 0, radius_wheel_spheres)))

SPHERES[2:, :2] = WHEEL_MOUNTING_POS  # Use the real mounting positions
N_SPHERES = SPHERES.shape[0]
SPHERES_RAD = SPHERES[:, -1].copy()
SPHERES_POS = SPHERES.copy()
SPHERES_POS[:, -1] = 1

SPHERES_F_IDX = np.zeros(N_SPHERES, dtype=int)

# Driving velocity
MAX_STEERING_VEL = 5.0  # rad / s
MAX_WHEEL_VEL = 2.0  # matrix / s


def calibrate_wheel_mounting_poss():
    pass
    # TODO ask Berthold for his code for calibrating those
    # square_distances = np.zeros(4)
    # square_distances[0] = np.linalg.norm(WHEEL_MOUNTING_POS[2] - WHEEL_MOUNTING_POS[3])  # 12
    # square_distances[1] = np.linalg.norm(WHEEL_MOUNTING_POS[1] - WHEEL_MOUNTING_POS[2])  # 32
    # square_distances[2] = np.linalg.norm(WHEEL_MOUNTING_POS[0] - WHEEL_MOUNTING_POS[1])  # 43
    # square_distances[3] = np.linalg.norm(WHEEL_MOUNTING_POS[3] - WHEEL_MOUNTING_POS[0])  # 14
    #
    # wheel_distances = np.zeros(4)
    # wheel_mounting0 = np.array([[+0.343, +0.257],  # front left
    #                             [+0.343, -0.257],  # front right
    #                             [-0.343, -0.257],  # back right
    #                             [-0.343, +0.257]])

# Forward offset of the head in get ready position
# x = 0.088  # FIXME write hm matrices to incorporate torso and head rotation
# x = 0.08914745
# this is not valid if the torso is in a completely different position
# TODO write your own fun functions so that this part can be computed directly
# + efficiently
