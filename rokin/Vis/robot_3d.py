import numpy as np

from wzk.pv.plotting import (pv, plot_spheres, plot_frames, plot_mesh, plot_bool_vol)
from wzk.pv.widgets import add_key_slider_widget, add_multiple_slider_widgets
from rokin.Robots import Justin19, JustinArm07, JustinFinger03, JustinHand12, JustinHand12Cal, JustinBase03  # noqa
from rokin.Robots.Justin19.justin19_primitives import justin_primitives


class RobotViz:
    def __init__(self, robot, q, show_frames=False, additional_frames=None, frames_scale=0.1,
                 mode='spheres', **kwargs):
        self.robot = robot
        self.q = q

        self.show_frames = show_frames
        self.frames_scale = frames_scale
        self.kwargs = kwargs
        self.mode = mode

        self.p = pv.Plotter()
        if additional_frames is not None:
            plot_frames(f=additional_frames, h=None, p=self.p, scale=self.frames_scale)

        self.h_spheres = None
        self.h_frames = None
        try:
            self.h_meshes = [None] * len(self.robot.meshes)
        except AttributeError:
            self.h_meshes = None

    def _update_frames(self, f):
        if self.show_frames is not None:
            tcp_f = f[self.show_frames, :, :]
            self.h_frames = plot_frames(f=tcp_f, h=self.h_frames, p=self.p, scale=self.frames_scale)

    def _update_spheres(self, v, i):
        self.q[i] = v
        f, x = self.robot.get_x_spheres(q=self.q, return_frames2=True)
        self.h_spheres = plot_spheres(x=x, r=self.robot.spheres_rad,
                                      h=self.h_spheres, p=self.p,
                                      color='blue', **self.kwargs)
        self._update_frames(f)

    def _update_meshes(self, v, i):
        self.q[i] = v
        f = self.robot.get_frames(q=self.q)
        plot_robot_meshes(f=f, robot=self.robot, p=self.p, h=self.h_meshes, **self.kwargs)
        self._update_frames(f)

    def update(self, v, i):
        if 'spheres' in self.mode:
            self._update_spheres(v, i)
        if 'meshes' in self.mode:
            self._update_meshes(v, i)

    def add_multiple_slider_widgets(self, grid, idx, names):
        add_multiple_slider_widgets(p=self.p, ranges=self.robot.limits, grid=grid, idx=idx, names=names,
                                    callback=self.update, x0=self.q)


def plot_robot_meshes(f, robot,
                      p=None, h=None,
                      **kwargs):
    if h is None:
        h = [None] * len(robot.meshes)

    f_meshes = f[robot.meshes_f_idx]
    f_meshes = f_meshes @ robot.meshes_frames
    for i, (mm, ff) in enumerate(zip(robot.meshes, f_meshes)):
        h[i] = plot_mesh(m=mm, f=ff, h=h[i], p=p, **kwargs)

    return h


def create_grid(ll, ur, n, pad):
    ll, ur, n, pad = np.atleast_1d(ll, ur, n, pad)

    w = ur - ll
    s = (w - pad*(n-1))/n

    x = ll[0] + np.arange(n[0])*(s[0]+pad[0])
    y = ll[1] + np.arange(n[1])*(s[1]+pad[1])
    return (x, y), s


def justin_arm_07_interactive(show_frames=None, additional_frames=None,
                              mode='meshes',
                              **kwargs):

    robot = JustinArm07()
    q = np.zeros(robot.n_dof)

    grid = create_grid(ll=(0.05, 0.05), ur=(0.95, 0.22), n=(3, 3), pad=(-0.015, 0.05))
    names = ['SHOULDER ROT', 'SHOULDER UD',
             'ELBOW ROT', 'ELBOW LR',
             'WRIST ROT ', 'WRIST RL', 'WRIST DU']

    idx = np.empty((robot.n_dof, 2), dtype=int)
    idx[:2, 0] = np.arange(2)  # shoulder
    idx[:2, 1] = 2
    idx[2:4, 0] = np.arange(2)  # elbow
    idx[2:4, 1] = 1
    idx[4:7, 0] = np.arange(3)  # wrist
    idx[4:7, 1] = 0

    viz = RobotViz(robot=robot, q=q, show_frames=show_frames, additional_frames=additional_frames, frames_scale=0.05,
                   mode=mode, **kwargs)
    viz.add_multiple_slider_widgets(grid=grid, idx=idx, names=names)
    viz.p.show()


def justin_finger_03_interactive(show_frames=None, additional_frames=None,
                                 mode='meshes',
                                 **kwargs):

    robot = JustinFinger03()
    q = np.zeros(robot.n_dof)

    grid = create_grid(ll=(0.05, 0.05), ur=(0.95, 0.22), n=(3, 1), pad=(-0.015, 0.05))
    names = ['base_lr', 'base_ud', 'middle_ud']

    idx = np.zeros((robot.n_dof, 2), dtype=int)
    idx[:3, 0] = np.arange(3)

    viz = RobotViz(robot=robot, q=q, show_frames=show_frames, additional_frames=additional_frames, frames_scale=0.02,
                   mode=mode, **kwargs)
    viz.add_multiple_slider_widgets(grid=grid, idx=idx, names=names)
    viz.p.show()


def justin_hand_12_interactive(show_frames=None, additional_frames=None,
                               mode='meshes',
                               **kwargs):

    robot = JustinHand12()
    q = np.zeros(robot.n_dof)

    grid = create_grid(ll=(0.05, 0.05), ur=(0.95, 0.22), n=(4, 3), pad=(-0.015, 0.05))
    names = ['ring', '', '',
             'middle', '', '',
             'fore', '', '',
             'thumb', '', '']

    idx = np.zeros((robot.n_dof, 2), dtype=int)
    idx[:, 0] = np.repeat(range(4), 3)
    idx[:, 1] = np.tile(range(3), 4)

    viz = RobotViz(robot=robot, q=q, show_frames=show_frames, additional_frames=additional_frames, frames_scale=0.05,
                   mode=mode, **kwargs)
    viz.add_multiple_slider_widgets(grid=grid, idx=idx, names=names)
    viz.p.show()


def justin_19_interactive(start='getready', show_frames=None, additional_frames=None,
                          mode='meshes',
                          **kwargs):

    q = justin_primitives(justin=start).ravel()

    robot = Justin19()

    grid = create_grid(ll=(0.05, 0.05), ur=(0.95, 0.22), n=(10, 2), pad=(-0.015, 0.05))
    names = ['TORSO_PAN', 'TORSO_1', 'TORSO_2',
             'SHOULDER', 'SHOULDER', 'ELBOW', 'ELBOW', 'WRIST', 'WRIST', 'WRIST',
             'ROT', 'UP_DOWN', 'ROT', 'LEFT_RIGHT', 'ROT', 'RIGHT_LEFT', 'DOWN_UP',
             'HEAD_PAN', 'HEAD_TILT']

    idx = np.empty((robot.n_dof, 2), dtype=int)
    idx[:10, 0] = np.arange(10)  # torso + right
    idx[:10, 1] = 1
    idx[10:17, 0] = np.arange(3, 10)  # left
    idx[10:17, 1] = 0
    idx[17:19, 0] = np.arange(0, 2)  # head
    idx[17:19, 1] = 0

    viz = RobotViz(robot=robot, q=q, show_frames=show_frames, additional_frames=additional_frames, frames_scale=0.2,
                   mode=mode, **kwargs)
    viz.add_multiple_slider_widgets(grid=grid, idx=idx, names=names)
    viz.p.show()


# def justin_base_03(q, obstacle_img=None, voxel_size=None, overlap_img=None,
#                    additional_frames=None):
#     robot = JustinBase03()
#     raise NotImplementedError
#     # q = handle_q(q)
#     #
#     # n_waypoints = q.shape[1]
#     # robot = JustinBase03()
#     #
#     # class BasePath(HasTraits):
#     #     time_steps = Range(0, n_waypoints - 1, 0, mode='slider')
#     #     scene = Instance(MlabSceneModel, ())
#     #     h_spheres = None
#     #     tcp_frame_plot = None
#     #
#     #     @on_trait_change('time_steps, scene.activated')
#     #     def update_plot(self):
#     #         xq_t = q[:, self.time_steps:self.time_steps + 1, :]
#     #         x_spheres = forward.get_x_spheres(q=xq_t, robot=robot)
#     #         x = x_spheres[..., 0].ravel()
#     #         y = x_spheres[..., 1].ravel()
#     #         z = np.zeros_like(x)
#     #
#     #         if self.h_spheres is None:
#     #
#     #             self.h_spheres = self.scene.mlab.points3d(x, y, z, robot.spheres_rad, mode='sphere',
#     #                                                       scale_factor=2, resolution=50)
#     #
#     #             if obstacle_img is not None and obstacle_img.sum() > 0:
#     #                 world_size = np.array(obstacle_img.shape) * voxel_size
#     #                 self.scene.mlab.imshow(obstacle_img.astype(int),
#     #                                        extent=[0, world_size[0], 0, world_size[1], 0, 0],
#     #                                        interpolate=False, vmin=0.1, colormap='Greys')
#     #
#     #             self.waypoints = self.scene.mlab.points3d(q[..., 0].ravel(), q[..., 1].ravel(),
#     #                                                       np.zeros(n_waypoints))
#     #             self.waypoints.mlab_source.dataset.point_data.vectors = np.ones((n_waypoints, 3)) * 0 has no effect
#     #             self.waypoints.mlab_source.dataset.point_data.scalars = np.full(n_waypoints, 0.2)
#     #             if additional_frames is not None:
#     #                 self.h_spheres = self.scene.mlab.points3d(additional_frames[0], additional_frames[1], 0, 0.1)
#     #
#     #         else:
#     #             self.h_spheres.mlab_source.trait_set(x=x, y=y, z=z)
#     #
#     #     view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
#     #                      height=250, width=300, show_label=False),
#     #                 Group('time_steps', '_'),
#     #                 resizable=True)
#
#     # base_model = BasePath()
#     # base_model.configure_traits()


def robot_path_interactive(q, robot,
                           mode='spheres',
                           obstacle_img=None, voxel_size=None, img_mode='voxel',
                           show_frames=None,
                           additional_frames=None,
                           color=None, alpha=1.):

    n_waypoints, n_joints = q.shape
    f, x = robot.get_x_spheres(q=q[0], return_frames2=True)
    p = pv.Plotter()
    plot_bool_vol(p=p, img=obstacle_img, voxel_size=voxel_size, mode=img_mode, color='black')

    if mode == 'spheres':
        h = plot_spheres(p=p, x=x, r=robot.spheres_rad, opacity=alpha, color=color)
    elif mode == 'meshes':
        h = plot_robot_meshes(p=p, f=f, robot=robot, opacity=alpha, color=color)
    else:
        raise ValueError

    if additional_frames is not None:
        plot_frames(p=p, f=additional_frames, scale=1 / 3)
    if show_frames is not None:
        h_frames = plot_frames(p=p, f=f[show_frames], scale=1/3)

    def update(t):
        t = int(np.round(t))
        ff, xx = robot.get_x_spheres(q=q[t], return_frames2=True)

        if mode == 'spheres':
            plot_spheres(h=h, x=xx, r=robot.spheres_rad)
        elif mode == 'meshes':
            plot_robot_meshes(h=h, f=ff, robot=robot)
        else:
            raise ValueError

        if show_frames is not None:
            plot_frames(h=h_frames, f=ff[show_frames], scale=1/3)

    s = p.add_slider_widget(callback=update, value=0, rng=(0, n_waypoints-1), title='time steps', fmt="%.0f")
    add_key_slider_widget(p=p, slider=s, callback=update, step=1)
    p.show()


if __name__ == '__main__':

    # justin_19_interactive(start='zero', show_frames=[13, 22], opacity=1, mode='meshes')
    justin_arm_07_interactive(show_frames=[0, 7], opacity=1, mode='meshes')
    # justin_hand_12_interactive(show_frames=[0, 5, 11, 17, 23], opacity=0.9)
    # justin_finger_03_interactive(show_frames=[0], opacity=0.8)

    # import scipy.io
    # mat = scipy.io.loadmat('/Data/JustinHand19/export-1.mat')['data'][0, 0]
    # q0 = np.concatenate((mat[1], mat[2], mat[3], mat[4], mat[5], mat[6]))
    #
    # ixx = np.array([32, 31, 34, 59, 56, 46, 41, 13, 52, 22, 18, 23,  0, 43,  6, 35,  8,
    #                 26, 19, 14,  9, 40, 45, 53, 29, 33, 20,  3, 30, 38, 17, 16, 47, 24,
    #                 42,  4, 21, 44,  7, 48, 28, 54,  2,  5, 49, 37,  1, 57, 15, 39, 11,
    #                 12, 25, 27, 10, 50, 58, 36, 51, 55])
    #
    # q0 = q0[ixx]
    # q0[0] = np.array([0.3027871, -0.3161505, 0.6642020,
    #                   -0.0171715, 0.4152261, 0.8547665,
    #                   -0.4747111, 0.3777183, 0.1215826,
    #                   -0.0106201, -0.1294758, 0.9147583])
    # robot_path_interactive(q=q0, robot=JustinHand12(), mode='meshes')
    # robot_path_interactive(q=q0, robot=JustinHand12Cal(), mode='meshes')
