from matplotlib.animation import FuncAnimation

from wzk import repeat_dict
from wzk.mpl import *


def __x_wrapper(x):
    if x.ndim == 2:
        x = x[np.newaxis, :, :]
    return x


def __q_wrapper(q):
    if q.ndim == 2:
        q = q[np.newaxis, :, :]
    return q


def c_list_wrapper(c, n=20):
    c_list_dict = {'tum_blue443': [pallet_tum['blue_4'],
                                   pallet_tum['blue_4'],
                                   pallet_tum['blue_3']],
                   'tum_blue4432': [pallet_tum['blue_4'],
                                    pallet_tum['blue_4'],
                                    pallet_tum['blue_3'],
                                    pallet_tum['blue_2']],
                   'k_b_p_t_c_y': ['black', 'blue', 'xkcd:purple', 'xkcd:teal', 'cyan', 'yellow']}

    if c is None:
        c = 'tum_blue443'

    if c in c_list_dict:
        c_list = c_list_dict[c]
    elif isinstance(c, str):
        c_list = [c]
    else:
        raise ValueError
    c_list *= n // len(c_list) + 1
    return c_list


def new_world_fig(limits=None, n_dim=2,
                  ax_labels=False, title=None,
                  width=5,
                  position=None, monitor=-1,
                  **kwargs):

    fig, ax = new_fig(n_dim=n_dim, width=width, height=width, title=title, position=position, monitor=monitor, aspect=1,
                      **kwargs)

    set_ax_limits(ax=ax, limits=limits, n_dim=n_dim)
    turn_ticklabels_off(ax=ax)
    set_ticks_position(ax=ax, position='both')

    if ax_labels:
        set_labels(ax=ax, labels=('x', 'y', 'z'))

    return fig, ax


def plot_x_path(x, r=None, ax=None, h=None,
                **kwargs):
    n_dim = x.shape[-1]

    if r is not None:
        size_new = size_units2points(size=r, ax=ax)
        kwargs['markersize'] = size_new

    if h is None:
        h = ax.plot(*x.reshape(-1, n_dim).T, **kwargs)[0]

        if r is not None:
            size_units2points_listener(ax=ax, h=h, size=r)

        return h

    else:
        h.set_xdata(x[..., 0].ravel())
        h.set_ydata(x[..., 1].ravel())
        if n_dim == 3:
            h.set_3d_properties(x[..., 2].ravel())


def plot_spheres(q, robot,
                 ax=None, h=None,
                 **kwargs):
    x = robot.get_x_spheres(q=q)
    x = x.reshape((-1, x.shape[-1]))
    h = plot_circles(h=h, ax=ax, x=x, r=robot.spheres_rad, **kwargs)
    return h


def plot_x_spheres(*, x_spheres, ax=None, style_path=None, style_arm=None):
    style_path_default = {'marker': 'o',
                          'markersize': 10,
                          'linestyle': '--',
                          'linewidth': 2,
                          'alpha': 0.9,
                          'zorder': 20,
                          'color': c_list_wrapper(c=None, n=20)}

    style_arm_default = {'marker': 'o',
                         'markersize': 10,
                         'linestyle': '-',
                         'linewidth': 3,
                         'alpha': (0.7, 0.6, 0.7),
                         'zorder': 10,
                         'color': ('g', 'k', 'r')}

    if style_path is None:
        style_path = style_path_default

    if style_arm is None:
        style_arm = style_arm_default

    def index2start_middle_end(n):
        idx = np.ones(n)
        idx[0] = 0
        idx[-1] = 2
        return idx

    x_spheres = __x_wrapper(x=x_spheres)
    n_wp, n_spheres, n_dim = x_spheres.shape
    h_paths = []
    h_arms = []

    style_path = repeat_dict(style_path, n=n_spheres)
    style_arm = repeat_dict(style_arm, n=3)  # start, path, end

    # Path of spheres
    if n_wp > 1:
        for i in range(n_spheres):
            h_paths.append(plot_x_path(x=x_spheres[:, i, :], ax=ax, **style_path[i]))

    # Arm position at the different waypoints
    start_middle_end = index2start_middle_end(n_wp)
    if n_spheres > 1:
        for i in range(n_wp):
            h_arms.append(plot_x_path(x=x_spheres[i, :, :], ax=ax, **style_arm[start_middle_end[i]]))

    return h_paths, h_arms


def plot_x_spheres_update(*, h, x_spheres):
    h_paths, h_arms = h

    x_spheres = __x_wrapper(x=x_spheres)
    n_wp, n_spheres, n_dim = x_spheres.shape

    if n_wp > 1:
        for i in range(n_spheres):
            plot_x_path(x=x_spheres[:, i, :], h=h_paths[i])

    if n_spheres > 1:
        for i in range(n_wp):
            plot_x_path(x=x_spheres[i, :, :], h=h_arms[i])


def steering_quiver(ax, xy, radius, angles, **kwargs):
    return ax.quiver(xy[..., 0], xy[..., 1], np.zeros_like(radius), radius, angles=np.rad2deg(angles), pivot='mid',
                     width=np.mean(radius) / 20,
                     **kwargs)


def update_steering_quiver(h, xy, angles):
    h.angles = np.rad2deg(angles)
    h.set_offsets(xy)


def animate_mobile_base(q, robot, steering_angles, wheel_velocity):
    q = q.reshape(1, -1, q.shape[-1])
    x_spheres = robot.get_x_spheres(q=q, robot=robot)[0]

    wheel_direction = np.stack((np.cos(steering_angles), np.sin(steering_angles))).transpose((1, 2, 0))
    wheel_velocity = wheel_direction * wheel_velocity[:, :, np.newaxis]
    wheel_path = np.cumsum(np.concatenate((x_spheres[:1, 2:], wheel_velocity), axis=0), axis=0)

    limits = np.array([[0, 10],
                       [0, 10]])
    fig, ax = new_world_fig(limits=limits)
    fig.subplots_adjust(bottom=0.2)

    ax.plot(q[0, :, 0], q[0, :, 1], ls=':', color='k')
    for i in range(4):
        ax.plot(wheel_path[:, i, 0], wheel_path[:, i, 1], ls='-', color='r')

    for i in range(4):
        ax.plot(x_spheres[:, 2 + i, 0], x_spheres[:, 2 + i, 1], ls=':', color='b')

    h = steering_quiver(ax=ax, xy=x_spheres[0, 2:, :], radius=robot.spheres_rad[2:], angles=steering_angles[0],
                        zorder=100)

    def cb_steering(ii):
        update_steering_quiver(h, xy=x_spheres[ii, 2:], angles=steering_angles[ii])

    ax, slider = animate_sphere_robot_2d(q=q, robot=robot, ax=ax, callback=cb_steering)

    return ax, slider


def animate_sphere_robot_2d(q, robot, ax=None, callback=None, gif=False):
    q = q.reshape(-1, robot.n_dof)
    x_spheres = robot.get_x_spheres(q=q)
    n_wp = q.shape[0]

    spheres = DraggableCircleList(ax=ax, xy=x_spheres[0], radius=robot.spheres_rad,
                                  facecolor='xkcd:dark grey', edgecolor='xkcd:dark grey')

    def cb_slider(val):
        i = int(val)
        print(i)
        spheres.set_xy(xy=x_spheres[i])
        if callback is not None:
            callback(i)

    if gif:
        ax.figure.subplots_adjust(left=0.0, bottom=0.0, right=1, top=1.0, wspace=0.0, hspace=0.0)
        ax.axis('off')
        ani = FuncAnimation(ax.figure, func=cb_slider, frames=n_wp, repeat=False)
        ani.save(f"{gif}.mp4", writer='ffmpeg', fps=10, metadata=dict(artist='Johannes Tenhumberg'), dpi=300)

        key_slider = None
    else:
        # Slider
        key_slider = create_key_slider(ax=get_xaligned_axes(ax=ax, y_distance=0.1, height=0.03),
                                       label='Time Step', valfmt='%d',
                                       valmin=0, valmax=n_wp - 1, valinit=0, valstep=1,
                                       callback=cb_slider)

    return ax, key_slider


def draw_sphere_grid(q, robot, ax, **kwargs):
    x_sphere = robot.get_x_spheres(q=q.reshape(-1, robot.n_dof))
    ax.plot(*x_sphere.transpose(2, 1, 0), **kwargs)
    ax.plot(*x_sphere.transpose(2, 0, 1), **kwargs)


def animate():
    pass
    # axcolor = 'lightgoldenrodyellow'
    # ax_timesteps = plt.axes([0.2, 0.01, 0.6, 0.05], facecolor=axcolor)
    # s_time = Slider(ax=ax_timesteps, label='Time', valmin=0, valmax=n_wp - 1, valinit=0, valfmt='%0.0f')
    # x_warm = jb.get_x_spheres(x=q[:, :2], theta=q[:, -1:])[0, :, :, :]
    # print(x_warm.shape)
    # circle_list = []
    #
    # for s in range(jb.N_SPHERES):
    #     circle = patches.Circle(xy=x_warm[s, 0, :2], radius=jb.BASE_SPHERES_RADII[s], facecolor='r')
    #     circle_list.append(ax.add_patch(circle))
    #
    # def update(val, t_old=[0]):
    #     t = int(s_time.val)
    #     if t != t_old[0]:
    #         t_old[0] = t
    #         for s in range(jb.N_SPHERES):
    #             circle_list[s].center = x_warm[s, t, :2]
    #
    # #
    # s_time.on_changed(update)
    # plt.show()
    #
    # return s_time


def animate_arm(frames, ax):
    print('The disadvantage of quiver is that it does not work in 3D?')
    # TODO
    #   What is quicker, what looks better, what is easier,
    #   It tend towards spheres + lines
    # TODO IF quiver: add head_width and spheres to make it a little bit nicer
    radius = 0.1
    limb_lengths = 0.5

    n_waypoints, n_frames, _, _ = frames.shape

    i = 0
    xy = frames[i, :, :2, -1]
    uv = frames[i, :, :2, 0] * limb_lengths

    h = quiver(ax=ax, xy=xy, uv=uv,
               width=radius, color='k')

    # noinspection PyDefaultArgument
    def on_press(event, ii=[0]):
        # print(event.xdata, event.ydata)
        if event.key == 'left':
            ii[0] -= 1
        elif event.key == 'right':
            ii[0] += 1

        ii[0] %= n_waypoints

        _xy = frames[ii[0], :, :2, -1]
        _uv = frames[ii[0], :, :2, 0] * limb_lengths

        quiver(h=h, xy=_xy, uv=_uv)
        ax.get_figure().show()

    cid_key = ax.get_figure().canvas.mpl_connect('key_press_event', on_press)

    return ax, cid_key


def plot_arm(frames, ax):
    radius = 0.1
    limb_lengths = 0.5

    n_waypoints, n_frames, _, _ = frames.shape
    xy = frames[:, :, :2, -1]
    uv = frames[:, :, :2, 0] * limb_lengths

    h = quiver(ax=ax, xy=xy, uv=uv, width=radius, color='k')

    return h
