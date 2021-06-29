import importlib

import numpy as np

from rokin import forward
from rokin.CodeGeneration import main as code_generation

from wzk.math2 import angle2minuspi_pluspi
from wzk.random2 import random_uniform_ndim
from wzk.spatial.transform import initialize_frames, apply_eye_wrapper

_cpp_order = 'c'
_cpp_dtype = 'f8'  # 'f4'-> float, 'f8' -> double


def import_robot_cpp(robot, replace=False, verbose=1):
    try:
        return importlib.import_module(f'rokin.Robots.{robot.id}.cpp.{robot.id}')

    except ModuleNotFoundError:
        pass
    except ImportError:
        pass

    assert robot.n_dim == 3
    code_generation.generate_robot_cpp(robot=robot, replace=replace, verbose=verbose)
    code_generation.compile_robot_cpp(robot=robot, replace=replace, verbose=verbose)
    raise UserWarning(f"Successful code generation. Rerun your code to use the C++ backend for {robot.id}.")


class Robot(object):
    __slots__ = ('id',                     # str                       | Unique name of the robot
                 'n_dof',                  # int                       | Degrees of Freedom (dof)
                 'n_dim',                  # int                       | Spacial dimensions, 2D/3D
                 'n_frames',               # int                       | n_frames >= n_dof
                 'dh',                     # float[n_dof][4]           | DH parameters
                 'limits',                 # float[n_dof][2]           | (min, max) value for each dof
                 'next_frame_idx',         # int[n_frames][?]          | List of Lists indicating the next frame(s)
                 'prev_frame_idx',         # int[n_frames]             | Array indicating the previous frame
                 #                                                     | Only provide one as they are redundant
                 #                                                     | and the other is inferred automatically
                 'joint_frame_idx',        # int[n_frames][?]          | List of Lists showing the leverage point
                 'infinity_joints',        # bool[n_dof]               | Array indicating which joints are limitless
                 'f_world_robot',          # float[n_dim+1][n_dim+1]   | Position base frame in the world

                 'joint_frame_influence',  # bool[n_dof][n_frames]     | Matrix showing the influence of each joint
                 'frame_frame_influence',  # bool[n_frames][n_frames]  | Matrix showing the influence of each frame

                 # Sphere Model
                 'spheres_rad',            # float[n_spheres]          | Ras
                 'spheres_pos',            # float[n_spheres][n_dim+1] | The homogeneous coordinates of each sphere
                 #                                                     | with respect to its frame
                 'spheres_f_idx',          # int[n_spheres]            | The frame in which each sphere is fixed
                 'limb_lengths',           # float[n_frames-1]         | Length of each limb [m]  # TODO use DH for 2D

                 '_cpp'                    # module                    | C++ functions to calculate forward kinematics
                 )

    def __repr__(self):
        return f"{self.id[:-2]}: {self.n_dof} dof, {self.n_frames} frames"

    def inf_joint_wrapper(self, q):
        if self.infinity_joints is not None and np.sum(self.infinity_joints) > 0:
            q[..., self.infinity_joints] = angle2minuspi_pluspi(q[..., self.infinity_joints])
        return q

    def prune_joints2limits(self, q):
        q = self.inf_joint_wrapper(q)
        return np.clip(q, a_min=self.limits[:, 0], a_max=self.limits[:, 1])

    def get_frames(self, q):
        return self._get_frames_cpp(q)

    def get_frames_jac(self, q):
        return self._get_frames_jac_cpp(q)

    def get_frames_dh(self, q, dh):
        return self._get_frames_dh_cpp(q, dh)

    def get_frames_jac_dh(self, q, dh):
        return self._get_frames_jac_dh_cpp(q, dh)

    def sample_q(self, shape=None):
        return random_uniform_ndim(low=self.limits[:, 0], high=self.limits[:, 1], shape=shape)

    def get_x_spheres(self, q, return_frames2=False):
        return forward.get_x_spheres(q=q, robot=self, return_frames2=return_frames2)

    def get_x_spheres_jac(self, q, return_frames2=False):
        return forward.get_x_spheres_jac(q=q, robot=self, return_frames2=return_frames2)

    # init
    def _init_f(self, shape, mode='zero', dtype=None, order=None):
        return initialize_frames(shape=shape[:-1] + (self.n_frames,), n_dim=self.n_dim, mode=mode,
                                 dtype=dtype, order=order)

    def _init_j(self, shape, dtype=None, order=None):
        return np.zeros(shape[:-1] + (self.n_frames, self.n_dim+1, self.n_dim+1, self.n_dof),
                        dtype=dtype, order=order)

    # Helper for cpp backends
    def _init_f_cpp(self, shape):
        return self._init_f(shape=shape, dtype=_cpp_dtype, order=_cpp_order)

    def _init_j_cpp(self, shape):
        return np.zeros(shape[:-1] + (self.n_dof, self.n_frames, self.n_dim+1, self.n_dim+1),
                        dtype=_cpp_dtype, order=_cpp_order)

    @staticmethod
    def _2cpp(a):
        return a.astype(dtype=_cpp_dtype, order=_cpp_order)

    def _get_frames_cpp(self, q):
        f = self._init_f_cpp(q.shape)
        self._cpp.get_frames(frames=f, joints=self._2cpp(q), n=q[..., 0].size)

        return self.f_world_robot @ f

    def _get_frames_jac_cpp(self, q):
        f, j = self._init_f_cpp(q.shape), self._init_j_cpp(q.shape)
        self._cpp.get_frames_jacs(frames=f, jacs=j, joints=self._2cpp(q), n=q[..., 0].size)

        return apply_eye_wrapper(f, self.f_world_robot), np.moveaxis(j, -4, -1)

    def _get_frames_dh_cpp(self, q, dh):
        f = self._init_f_cpp(q.shape)
        if dh.ndim == 2:
            self._cpp.get_frames_dh4(frames=f, joints=self._2cpp(q),
                                     dh=self._2cpp(dh), n=q[..., 0].size)
        else:
            if dh.shape[-1] == 4:
                self._cpp.get_frames_dh4b(frames=f, joints=self._2cpp(q),
                                          dh=self._2cpp(dh), n=q[..., 0].size)
            else:
                self._cpp.get_frames_dh5b(frames=f, joints=self._2cpp(q),
                                          dh=self._2cpp(dh), n=q[..., 0].size)

        return self.f_world_robot @ f

    def _get_frames_jac_dh_cpp(self, q, dh):
        f, j, = self._init_f_cpp(q.shape), self._init_j_cpp(q.shape)
        if dh.ndim == 2:
            self._cpp.get_frames_jacs_dh4(frames=f, jacs=j, joints=self._2cpp(q),
                                          dh=self._2cpp(dh), n=q[..., 0].size)
        else:
            if dh.shape[-1] == 4:
                self._cpp.get_frames_jacs_dh4b(frames=f, jacs=j, joints=self._2cpp(q),
                                               dh=self._2cpp(dh), n=q[..., 0].size)
            else:
                self._cpp.get_frames_jacs_dh5b(frames=f, jacs=j, joints=self._2cpp(q),
                                               dh=self._2cpp(dh), n=q[..., 0].size)

        return apply_eye_wrapper(f, self.f_world_robot), np.moveaxis(j, -4, -1)
