import numpy as np

# Masses [kg]
masses_torso = np.array([4.651, 5.470, 2.925, 5.759])
masses_arm = np.array([2.6370, 2.326, 2.239, 2.204, 1.101, 1.641, 0.675])
masses_head = np.array([3.])
masses_hand = np.array([2.])

# Center of Gravity relative to frame [matrix]
pos_torso = np.array([[+1.3342422e+00, +1.5386544e+01, +7.4260405e+0],
                      [+2.1859133e+02, -1.3019641e+01, +6.2231342e+00],
                      [+1.5320139e+02, -2.2720538e+01, +1.9282567e+00],
                      [+5.3666155e+01, +7.4570502e+01, +6.8257189e-01]]) / 1000  # matrix

pos_arm_right = np.array([[-4.1410813e-01, +2.0290376e+01, -4.5943169e+01],
                          [-2.9078800e-02, +1.1406515e+02, +1.6447191e+01],
                          [-3.0257825e-02, +1.6511890e+01, -1.1329461e+02],  # x -> +x-z+y
                          [-3.0879429e-02, +1.1374485e+02, +1.6369065e+01],  #
                          [+1.0916025e-01, +2.1900792e+01, -1.0562437e+02],  #
                          [+4.9984737e-01, -7.8452020e+00, -8.7059954e+00],  #
                          [+2.9083180e+01, +7.8114352e+01, +1.9575027e+01]   # x -> +y-x+z
                          # [+2000., +3000., +4000.]   # x -> +y-x+z
                          ]) / 1000  # matrix

pos_arm_left = np.array([[+4.1410813e-01, -2.0290376e+01, +4.5943169e+01],  # x -> -x-y-z
                         [+2.9078800e-02, -1.1406515e+02, +1.6447191e+01],  # x -> -x-y+z
                         [-3.0257825e-02, -1.6511890e+01, +1.1329461e+02],  # x -> +x-y-z
                         [-3.0879429e-02, -1.1374485e+02, -1.6369065e+01],  # x -> +x-y-z
                         [-1.0916025e-01, +2.1900792e+01, +1.0562437e+02],  # x -> -x+y-z
                         [+4.9984737e-01, -7.8452020e+00, +8.7059954e+00],  # x
                         [+2.9083180e+01, +7.8114352e+01, -1.9575027e+01]   # x -> +y-x-z
                         # TODO Mirror symmetry in x or not ? ,right now symmetric
                         # [+2000., +3000., -4000.]   # x -> +y-x-z  # TODO Mirror symmetry ?
                         ]) / 1000  # matrix

pos_head = np.array([[0.01, -0.14, 0.]])
pos_hand = np.array([[0., 0., 0.10]])

MASS_POS = np.concatenate([pos_torso,
                           pos_arm_right, pos_hand,
                           pos_arm_left, pos_hand,
                           pos_head])
MASS_POS = np.concatenate([MASS_POS, np.ones((len(MASS_POS), 1))], axis=1)

MASSES = np.concatenate([masses_torso,
                         masses_arm, masses_hand,
                         masses_arm, masses_hand,
                         masses_head])

MASS_F_IDX = np.array([1, 2, 3, 4,                  # Torso
                       6, 7, 8, 9, 10, 11, 12,      # Right Arm
                       13,                          # Right Hand
                       15, 16, 17, 18, 19, 20, 21,  # Left Arm
                       22,                          # Left Hand
                       25])                         # Head


def approx_mass_model_old():
    pass
    # MASS_POS = np.array([[0.15, 0., 0., 1.],
    #                           [0.15, 0., 0., 1.],
    #                           [0.05, 0.07, 0., 1.],
    #
    #                           [0., 0., -0.08, 1.],
    #                           [0., 0.20, 0., 1.],  # Changed from the sphere model
    #                           [0., 0.20, 0., 1.],  # upper and lower arm are each 40cm long -> center 20 cm
    #                           [0., 0., 0.13, 1.],
    #
    #                           [0., 0., -0.08, 1.],
    #                           [0., -0.20, 0., 1.],
    #                           [0., -0.20, 0., 1.],
    #                           [0., 0., 0.13, 1.],
    #
    #                           [0.01, -0.14, 0., 1.]])
    #
    # MASS_F_IDX = np.array([2, 3, 4,         # Torso
    #                            5, 7, 9, 13,     # Arm Right
    #                            14, 16, 18, 22,  # Arm Left
    #                            25])             # Head
    #
    # TORQUE_F_IDX = np.array([1, 2, 3, 4,
    #                              6, 7, 8, 9, 10, 11, 12,
    #                              15, 16, 17, 18, 19, 20, 21,
    #                              24, 25])
    #
    # MASSES = np.array([10, 10, 10,
    #                    5, 5, 5, 2,
    #                    5, 5, 5, 2,
    #                    3])
