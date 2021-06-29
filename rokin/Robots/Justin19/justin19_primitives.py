import numpy as np

from wzk.spatial.transform import trans_euler2frame
from rokin.Robots.Justin19 import justin19_par as jtp

"""Return everything in radians"""


def torso_primitives(pose=None):
    q = np.zeros(3)
    if pose is None or pose == 'zero':
        q[:] = [0, 0, 15]
    elif pose == 'zero_right':
        q[:] = [-90, 0, 15]
    elif pose == 'zero_left':
        q[:] = [+90, 0, 15]

    elif pose == 'getready':
        q[:] = [0, -48.527, 97.386]
    elif pose == 'getready_right':
        q[:] = [-90, -48.527, 97.386]
    elif pose == 'getready_left':
        q[:] = [+90, -48.527, 97.386]

    elif pose == 'tube_grasp':
        q[:] = [-0.014, -48.703, 98.050]
    elif pose == 'tube_grasp_right':
        q[:] = [-90.014, -48.703, 98.050]
    elif pose == 'tube_grasp_left':
        q[:] = [+89.986, -48.703, 98.050]

    else:
        raise ValueError(f"Arm pose '{pose}' not known")

    return np.deg2rad(q)


def arm_primitives(pose=None):
    q = np.zeros(7)

    # General
    if pose is None or pose == 'zero':
        q[:] = [0, 0,
                0, 0,
                0, 0, 0]

    elif pose == 'getready':
        q[:] = [-60.532, -107.672,
                -26.066, 106.409,
                -5.171, -41.724, 9.236]

    elif pose == 'tube_grasp_low':  # The height of the table is 72.5cm and the tubes are 50cm long
        q[:] = np.array([0.06190796196460724, -2.0324809551239014,
                         -0.09767785668373108, 1.2669713497161865,
                         0.08680056780576706, -0.5265905261039734, 0.42132243514060974])
        q = np.rad2deg(q)

    elif pose == 'tube_grasp_high':
        q[:] = np.array([0.1827741414308548, -1.3087267875671387,
                         -0.4342576265335083, 1.546159267425537,
                         0.7625257968902588, -0.541204571723938, 0.35483649373054504])
        q = np.rad2deg(q)

    # Side
    elif pose == 'side_vertical':
        q[:] = [0, 60,  # +90 degree from horizontal
                0, 0,
                0, 0, 0]

    elif pose == 'side_up':
        q[:] = [0, 30,  # +60 degree from horizontal
                0, 0,
                0, 0, 0]
    elif pose == 'side_horizontal':
        q[:] = [0, -30,
                0, 0,
                0, 0, 0]
    elif pose == 'side_down_b':
        q[:] = [0, -80,  # -50 degree from horizontal
                0, 0,
                0, 0, 0]
    elif pose == 'side_down':
        q[:] = [0, -90,  # -60 degree from horizontal
                0, 0,
                0, 0, 0]

    elif pose == 'side_safety_step':
        q[:] = [-60.532, -77,
                24, 106.409,
                -5.171, -41.724, 9.236]
    elif pose == 'side_safety_step2':
        q[:] = [0, -77,
                -16, 106.409,
                -5.171, -41.724, 9.236]
    # Front
    elif pose == 'front_horizontal':
        q[:] = [-90, 90,
                0, 0,
                0, 0, 0]
    elif pose == 'front_up':
        q[:] = [-40, 55,
                0, 0,
                0, 0, 0]
    elif pose == 'front_down':
        q[:] = [-152, 113,
                0, 0,
                0, 0, 0]

    # Back
    elif pose == 'back_horizontal':
        q[:] = [90, 90,
                0, 0,
                0, 0, 0]
    elif pose == 'back_up':
        q[:] = [41, 66,
                0, 0,
                0, 0, 0]
    elif pose == 'back_down':
        q[:] = [133, 106,
                0, 0,
                0, 0, 0]

    # Around body
    elif pose == 'shield_body_front1':
        q[:] = [85, -105,
                120, -80,
                0, 0, 0]
    elif pose == 'shield_body_front2':
        q[:] = [90, -100,
                -55, 75,
                0, 0, 0]
    elif pose == 'shield_body_back':
        q[:] = [-85, -105,
                60, -80,
                0, 0, 0]

    elif pose == 'torero_out':
        q[:] = [10, -70,
                -40, 55,
                0, 0, 0]
    elif pose == 'torero_out2':
        q[:] = [40, -90,
                -60, -50,
                0, -45, 0]
    elif pose == 'torero_in':
        q[:] = [10, -70,
                -40, 55,
                0, 0, 0]
    else:
        raise ValueError(f"Arm pose '{pose}' not known")

    return np.deg2rad(q)


def head_primitives(pose):
    q = np.zeros(2)
    if pose is None or pose == 'zero' or pose == 'getready':
        q[:] = [0, 0]
    else:
        raise ValueError(f"Arm pose '{pose}' not known")
    return np.deg2rad(q)


def justin_primitives(*, justin=None,
                      torso=None, right_arm=None, left_arm=None, head=None):
    if justin is not None:
        if justin == 'getready':
            torso = right_arm = left_arm = head = 'getready'
        elif justin == 'getready_right':
            torso = 'getready_right'
            right_arm = left_arm = head = 'getready'
        elif justin == 'getready_left':
            torso = 'getready_left'
            right_arm = left_arm = head = 'getready'

        elif justin == 'zero':
            torso = right_arm = left_arm = head = 'zero'
        elif justin == 'zero_right':
            torso = 'zero_right'
            right_arm = left_arm = head = 'zero'
        elif justin == 'zero_left':
            torso = 'zero_left'
            right_arm = left_arm = head = 'zero'
        elif justin == 'zero_horizontal':
            torso = head = 'zero'
            right_arm = left_arm = 'side_horizontal'

        elif justin == 'getready_left_side_down':
            torso = right_arm = head = 'getready'
            left_arm = 'side_down_b'

        elif justin == 'getready_right_side_down':
            torso = left_arm = head = 'getready'
            right_arm = 'side_down_b'

        elif justin == 'tube_grasp':
            torso = 'tube_grasp'
            right_arm = 'tube_grasp_low'
            left_arm = 'tube_grasp_high'

        elif justin == 'tube_grasp_right':
            torso = 'tube_grasp_right'
            right_arm = 'tube_grasp_low'
            left_arm = 'tube_grasp_high'

        elif justin == 'tube_grasp_left':
            torso = 'tube_grasp_left'
            right_arm = 'tube_grasp_low'
            left_arm = 'tube_grasp_high'
        else:
            raise ValueError("Justin pose '{}' not known".format(justin))

    q = np.zeros(19)
    q[0:3] = torso_primitives(pose=torso)
    q[3:10] = arm_primitives(pose=right_arm)
    q[10:17] = arm_primitives(pose=left_arm)
    q[17:19] = head_primitives(pose=head)

    # Two arms around body
    # Obstacle on table
    # Obstacle wave
    return q


def example_path_justin(motion):
    """
    getup, side_wave, twist_dance1, twist_dance2, cut_arm
    """

    if motion == 'getup':
        q_start = justin_primitives(torso='getready', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero', right_arm='zero', left_arm='zero')
    elif motion == 'getup_l' or motion == 'getup_left':
        q_start = justin_primitives(torso='getready_left', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero_left', right_arm='zero', left_arm='zero')
    elif motion == 'getup_lr':
        q_start = justin_primitives(torso='getready_left', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero_right', right_arm='zero', left_arm='zero')
    elif motion == 'getup_lc':
        q_start = justin_primitives(torso='getready_left', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero', right_arm='zero', left_arm='zero')
    elif motion == 'getup_r' or motion == 'getup_right':
        q_start = justin_primitives(torso='getready_right', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero_right', right_arm='zero', left_arm='zero')
    elif motion == 'getup_rl':
        q_start = justin_primitives(torso='getready_right', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero_left', right_arm='zero', left_arm='zero')
    elif motion == 'getup_rc':
        q_start = justin_primitives(torso='getready_right', right_arm='getready', left_arm='getready')
        q_end = justin_primitives(torso='zero', right_arm='zero', left_arm='zero')

    elif motion == 'getdown':
        q_start = justin_primitives(torso='zero', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_l' or motion == 'getdown_left':
        q_start = justin_primitives(torso='zero_left', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready_left', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_lr':
        q_start = justin_primitives(torso='zero_left', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready_right', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_lc':
        q_start = justin_primitives(torso='zero_left', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_r' or motion == 'getdown_right':
        q_start = justin_primitives(torso='zero_right', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready_right', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_rl':
        q_start = justin_primitives(torso='zero_right', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready_left', right_arm='getready', left_arm='getready')
    elif motion == 'getdown_rc':
        q_start = justin_primitives(torso='zero_right', right_arm='zero', left_arm='zero')
        q_end = justin_primitives(torso='getready', right_arm='getready', left_arm='getready')

    elif motion == 'side_wave':
        q_start = justin_primitives(torso='zero', right_arm='side_up', left_arm='zero')
        q_end = justin_primitives(torso='zero', right_arm='side_down', left_arm='zero')

    elif motion == 'twist_dance1':  # one arm
        q_start = justin_primitives(torso='zero', right_arm='shield_body_front1', left_arm='zero')
        q_end = justin_primitives(torso='zero', right_arm='shield_body_back', left_arm='zero')

    elif motion == 'twist_dance2':
        q_start = justin_primitives(torso='zero', right_arm='shield_body_front1', left_arm='shield_body_back')
        q_end = justin_primitives(torso='zero', right_arm='shield_body_back', left_arm='shield_body_front1')

    elif motion == 'cut_arm':
        q_start = justin_primitives(torso='zero', right_arm='shield_body_front2', left_arm='front_up')
        q_end = justin_primitives(torso='zero', right_arm='shield_body_front2', left_arm='front_down')

    else:
        raise ValueError("Motion '{}' is not known".format(motion))

    return q_start, q_end


def right_arm_frames(location, rotation):

    if isinstance(location, (str, tuple, list)):

        if 'far-front' in location:
            x = 1
        elif 'front' in location:
            x = 0.5
        elif 'far-back' in location:
            x = -1
        elif 'back' in location:
            x = -0.5
        else:
            x = 0.0

        if 'far-left' in location:
            y = 1
        elif 'left' in location:
            y = 0.5
        elif 'far-right' in location:
            y = -1
        elif 'right' in location:
            y = -0.5
        else:
            y = 0.0

        if 'high' in location:
            z = 1.5
        elif 'low' in location:
            z = 0.5
        else:
            z = 1.0

        location = np.array([x, y, z])

    if isinstance(rotation, (str, tuple, list)):
        # x-axis: hand palm
        # y-axis: thumb
        # z-axis: fingers
        if rotation == 'forward':
            rotation = [np.pi / 2, np.pi / 2, -np.pi / 2]  # palm in negative z-direction
        elif rotation == 'backward':
            rotation = [-np.pi / 2, np.pi / 2, -np.pi / 2]  # palm in negative z-direction
        elif rotation == 'left':
            rotation = [0, -np.pi / 2, np.pi / 2]  # palm in negative z-direction
        elif rotation == 'right':
            rotation = [0, np.pi / 2, -np.pi / 2]  # palm in negative z-direction
        elif rotation == 'up':
            rotation = [-np.pi / 2, 0, 0]  # palm in negative y-direction
        # elif rotation == 'up':
        #     rotation = [+np.pi/2, 0, 0]  # palm in positive y-direction
        elif rotation == 'down':
            rotation = [0, np.pi, -np.pi / 2]  # palm in positive y-direction
        elif rotation == 'get_ready':
            rotation = []
        elif rotation == 'zero':
            rotation = [0, 60 * np.pi / 180, -np.pi / 2]
        elif rotation == 'waiter':
            raise NotImplementedError
            # rotation = []

        elif rotation == 'grasp_front_flat':
            rotation = [np.pi / 2, np.pi / 2 + 30 * np.pi / 180, -np.pi / 2]
        elif rotation == 'grasp_front_steep':
            rotation = [np.pi / 2, np.pi / 2 + 60 * np.pi / 180, -np.pi / 2]
        elif rotation == 'grasp_right_flat':
            rotation = [0, np.pi / 2 + 30 * np.pi / 180, -np.pi / 2]
        elif rotation == 'grasp_right_steep':
            rotation = [0, np.pi / 2 + 60 * np.pi / 180, -np.pi / 2]
        else:
            raise NotImplementedError
        rotation = np.array(rotation)

    return trans_euler2frame(trans=location, euler=rotation)


def get_free_joints(torso=False, right=False, left=False, head=False):
    """
    Define which joints should be fixed and which should move freely.
    """
    b_joints = np.zeros(jtp.N_JOINTS, dtype=bool)
    b_joints[jtp.IDX_J_TORSO] = torso
    b_joints[jtp.IDX_J_RIGHT] = right
    b_joints[jtp.IDX_J_LEFT] = left
    b_joints[jtp.IDX_J_HEAD] = head
    return b_joints

# Tube Grasp
# np.array([-1.5710337162017822, -0.8500221371650696, 1.7112914323806763,
#           0.06190796196460724, -2.0324809551239014,
#           -0.09767785668373108, 1.2669713497161865,
#           0.08680056780576706, -0.5265905261039734, 0.42132243514060974,
#           0.1827741414308548, -1.3087267875671387,
#           -0.4342576265335083, 1.546159267425537,
#           0.7625257968902588, -0.541204571723938, 0.35483649373054504,
#           -0.00011198032734682783, 2.6284658815711737e-05])
