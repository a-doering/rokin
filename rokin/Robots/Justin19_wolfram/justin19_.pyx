# Cython: infer_types=True
import cython
import numpy as np

# noinspection PyUnresolvedReferences
cimport forward_kin  # .pxd-file which links the external C-library for the kinematics of Justin


# Total number of joints and frames
cdef int MAX_NUM_OF_JOINTS = 19
cdef int MAX_NUM_OF_LINKS = 32  # Links == frames

cdef int N_DIM = 3
cdef int N_JOINTS = 19
cdef int N_FRAMES = 27


# Initialize Mathematica / WolframAlpha Library
forward_kin.crkw_init()


def __get_frames(double[:] q):
    cdef double[:, :, ::1] frames = np.zeros((MAX_NUM_OF_LINKS, 4, 4))
    forward_kin.crkw_robot_kinematics(<double*> &q[0],
                                      <double(*)[4][4]> &frames[0, 0, 0])
    return np.asarray(frames)


def __get_jac(double[:] q):
    cdef double[:, :, :, ::1] jac = np.zeros((MAX_NUM_OF_JOINTS, MAX_NUM_OF_LINKS, 4, 4))
    forward_kin.crkw_drobot_kinematics(<double*> &q[0],
                                       <double(*)[32][4][4]> &jac[0, 0, 0, 0])
    return np.asarray(jac)


def __get_frames_jac(double[:] q):
    cdef double[:, :, ::1] frames = np.zeros((MAX_NUM_OF_LINKS, 4, 4))
    cdef double[:, :, :, ::1] jac = np.zeros((MAX_NUM_OF_JOINTS, MAX_NUM_OF_LINKS, 4, 4))
    forward_kin.crkw_robot_kinematics(<double*> &q[0],
                                      <double(*)[4][4]> &frames[0, 0, 0])
    forward_kin.crkw_drobot_kinematics(<double*> &q[0],
                                       <double(*)[32][4][4]> &jac[0, 0, 0, 0])

    return np.asarray(frames), np.asarray(jac)


@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing
@cython.nonecheck(False)
def get_frames(double[:, :, :] q,
                 int n_samples,
                 int n_wp):
    """
    frame (n_samples, n_wp, n_links, n_dim+1, n_dim+1)
    """

    # Initialization
    # Temp variables for the C-function call
    cdef double[::1] q_temp = np.zeros(MAX_NUM_OF_JOINTS)
    cdef double[:, :, :, :, ::1] frames = np.zeros((n_samples, n_wp, MAX_NUM_OF_LINKS, 4, 4))

    # Loop over all joints and fill the values of the c-fun functions into the arrays
    for n in range(n_samples):
        for wp in range(n_wp):
            q_temp[:N_JOINTS] = q[n, wp, :]
            forward_kin.crkw_robot_kinematics(<double *> &q_temp[0],  # TODO Attention look up at memory addresses that are not initialized
                                              <double(*)[4][4]> &frames[n, wp, 0, 0, 0])

    return np.asarray(frames[:, :, :N_FRAMES, :, :])



@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing
@cython.nonecheck(False)
def get_frames_jac(double[:, :, :] q,
                   int n_samples,
                   int n_wp):

    """
    frame (n_samples, n_wp, n_links, n_dim+1, n_dim+1)
    frame_jac (n_samples, n_wp, n_joints, n_links, n_wp, n_dim+1, n_dim+1)
    """

    # Initialization
    # Temp variables for the C-function call
    cdef double[::1] q_temp = np.zeros(MAX_NUM_OF_JOINTS)

    cdef double[:, :, :, :, ::1] frames = np.zeros((n_samples, n_wp, MAX_NUM_OF_LINKS, 4, 4))
    cdef double[:, :, :, :, :, ::1] jac = np.zeros((n_samples, n_wp, MAX_NUM_OF_JOINTS, MAX_NUM_OF_LINKS, 4, 4))

    # Loop over all joints and fill the values of the c-fun functions into the arrays
    for n in range(n_samples):
        for wp in range(n_wp):
            q_temp[:N_JOINTS] = q[n, wp, :]
            forward_kin.crkw_robot_kinematics(<double *> &q_temp[0],
                                              <double(*)[4][4]> &frames[n, wp, 0, 0, 0])
            forward_kin.crkw_drobot_kinematics(<double *> &q_temp[0],
                                               <double(*)[32][4][4]> &jac[n, wp, 0, 0, 0, 0])

    return np.asarray(frames[:, :, :N_FRAMES, :, :]), np.asarray(jac[:, :, :N_JOINTS, :N_FRAMES, :, :])
