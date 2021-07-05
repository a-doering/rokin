import numpy as np
from matplotlib.animation import FuncAnimation

from wzk.mpl import new_fig, save_fig, plt, collections, geometry, patches, get_aff_trafo, FancyBbox
from wzk.strings import uuid4
from wzk.spatial.transform_2d import trans_theta2frame

from rokin.Robots import StaticArm


def get_body_kwargs():
    head = dict(width=0.15, height=0.17,
                zorder=2, facecolor='0.2', edgecolor='k', alpha=1.0, hatch='')
    chest = dict(width=0.40, height=0.2,
                 boxstyle='round4',
                 zorder=0, facecolor='0.4', edgecolor='k', alpha=1.0, hatch='')
    arms = dict(length=[0.30, 0.27], width=[0.1, 0.1],
                boxstyle='round4',
                zorder=1, facecolor='0.5', edgecolor='k', alpha=1.0, hatch='')
    hands = dict(length=[0.21, 0.14], width=[0.03, 0.03], angle=[-0.2, +0.2],
                 boxstyle='round4',
                 zorder=2, facecolor='0.4', edgecolor='k', alpha=1.0, hatch='')
    joints = dict(radius=0.055,
                  zorder=2, facecolor='0.2', edgecolor='k', alpha=1.0, hatch='')

    return head, chest, arms, hands, joints


def plot_robot(xy, q, chest, arms, hands, joints, head=None, ax=None):

    if ax is None:
        ax = plt.gca()

    q_c, (q_ra, q_la) = q[0], np.split(q[1:], 2)
    n = len(q_la) - 1

    if chest is not None:
        chest_width = chest.pop('width')
        chest_height = chest.pop('height')
        chest_boxstyle = chest.pop('boxstyle')
        pad = chest_width*0.1
        chest = FancyBbox(width=chest_width, height=chest_height,
                          boxstyle=chest_boxstyle, pad=pad,
                          transform=get_aff_trafo(xy1=xy, por=(chest_width/2, chest_height/2),
                                                  theta=np.rad2deg(q_c), ax=ax),
                          **chest)
        ax.add_patch(chest)

    else:
        chest_width = 0

    arm_length = arms.pop('length')
    arm_width = arms.pop('width')
    arm_boxstyle = arms.pop('boxstyle')
    assert len(arm_length) == n
    assert len(arm_width) == n

    robot = StaticArm(n_dof=4, limb_lengths=np.array([chest_width / 2] + arm_length + [1]))
    robot.f_world_robot = np.eye(3)

    frame_ra, frame_la = get_frames2(xy=xy, q_c=q_c, q_ra=q_ra, q_la=q_la, robot=robot)
    xy_ra, a_ra = frame_ra[1:-1, :2, -1], np.arctan2(*frame_ra[1:-1, 1::-1, 0].T),
    xy_la, a_la = frame_la[1:-1, :2, -1], np.arctan2(*frame_la[1:-1, 1::-1, 0].T)

    # Arms
    pad = 0.07
    arms_patches = [FancyBbox(width=arm_length[i % n], height=arm_width[i % n],
                              boxstyle=arm_boxstyle, pad=arm_length[i % n]*pad,
                              transform=get_aff_trafo(xy1=_xy,
                                                      por=(0, arm_width[i % n]/2), theta=np.rad2deg(_a), ax=ax),
                              **arms)
                    for i, (_xy, _a) in enumerate(zip(np.concatenate([xy_ra[:-1], xy_la[:-1]]),
                                                      np.concatenate([a_ra[:-1], a_la[:-1]])))]

    # ax.add_collection(collections.PatchCollection(arms_patches, **arms))
    # TODO https://stackoverflow.com/questions/64083563/adding-patchcollection-with-affine-transformations
    for p in arms_patches:
        ax.add_patch(p)

    # Joints
    if joints is not None:
        joints_radius = joints.pop('radius')

        joint_patches = [patches.Circle(xy=xy, radius=joints_radius) for xy in np.concatenate([xy_ra[:-1], xy_la[:-1]])]
        ax.add_collection(collections.PatchCollection(joint_patches, **joints))

    # Head
    if head is not None:
        head_patch = patches.Ellipse(xy=xy, width=head.pop('width'), height=head.pop('height'), angle=np.rad2deg(q_c),
                                     **head)
        ax.add_patch(head_patch)

    # Hands
    if hands is not None:
        hand_length = hands.pop('length')
        hand_width = hands.pop('width')
        hand_angle = hands.pop('angle')
        hand_boxstyle = hands.pop('boxstyle')
        try:
            hand_frame = hands.pop('coord_frame')
        except KeyError:
            hand_frame = None

        hl = np.array(hand_length)[[0, 1, 0, 1]]
        hw = np.array(hand_width)[[0, 1, 0, 1]]
        ha = np.array(hand_angle)[[0, 1, 1, 0]]
        ha = np.rad2deg(ha + np.array([a_ra[-1], a_ra[-1], a_la[-1], a_la[-1]]))
        hx = [xy_ra[-1], xy_ra[-1], xy_la[-1], xy_la[-1]]
        pad = max(hand_length) * 0.06

        hand_patches = [FancyBbox(width=_hl, height=_hw, boxstyle=hand_boxstyle, pad=pad,
                                  transform=get_aff_trafo(xy1=_hx, por=(0, _hw/2), theta=_ha, ax=ax),
                                  **hands)
                        for _hl, _hw, _ha, _hx in zip(hl, hw, ha, hx)]
        # ax.add_collection(collections.PatchCollection(hand_patches, **hands))
        for p in hand_patches:
            ax.add_patch(p)

        if hand_frame is not None:

            def get_dcm(_a, _xy, _hand_a, switch):
                _a += _hand_a
                dcm = trans_theta2frame(theta=_a - np.pi / 2)[:-1, :-1] * switch
                x = np.array([_xy[0] + np.cos(_a) * hand_length[0] / 2,
                              _xy[1] + np.sin(_a) * hand_length[0] / 2]) + dcm[:, 0] * hand_width[0] / 2
                return x, dcm

            def plot_coord_frame(_xy, dcm):
                arrow_length = hand_length[0]/2*0.9
                geometry.plot_coordinate_frame(ax=ax, x=_xy, dcm=dcm*arrow_length, mode='relative_fancy',
                                               width=0.1, marker='o')

            def plot_target_diamond(_xy, dcm):
                arrow_length = hand_length[0]/2 * 0.15
                ax.add_patch(patches.FancyArrow(x=_xy[0], y=_xy[1],
                                                dx=dcm[0, 0]*arrow_length, dy=dcm[1, 0]*arrow_length,
                                                length_includes_head=True, overhang=-1, head_length=arrow_length/2,
                                                head_width=arrow_length*4,
                                                color='k'))

            right_target, dcm_r = get_dcm(_xy=xy_ra[-1], _a=a_ra[-1], _hand_a=hand_angle[0], switch=+1)
            left_target, dcm_l = get_dcm(_xy=xy_la[-1], _a=a_la[-1], _hand_a=hand_angle[1], switch=-1)

            if hand_frame == 'diamond':
                plot_target_diamond(_xy=right_target, dcm=dcm_r)
                plot_target_diamond(_xy=left_target, dcm=dcm_l)
            elif hand_frame == 'frame':
                plot_coord_frame(_xy=right_target, dcm=dcm_r)
                plot_coord_frame(_xy=left_target, dcm=dcm_l)
            else:
                raise ValueError
            # right_target = plot_coord_frame(_xy=xy_ra[-1], _a=a_ra[-1], _hand_a=hand_angle[0], switch=+1)
            # left_target = plot_coord_frame(_xy=xy_la[-1], _a=a_la[-1], _hand_a=hand_angle[1], switch=-1)
            return right_target, left_target


def get_frames2(xy, q_c, q_ra, q_la, robot):
    robot.f_world_robot[:2, -1] = xy
    frame_ra = robot.get_frames(q=np.hstack([q_c, q_ra]))
    frame_la = robot.get_frames(q=np.hstack([q_c-np.pi, q_la]))
    return frame_ra, frame_la


def test():
    n = 5
    fig, ax = new_fig(aspect=1, width=n, height=n)
    ax.set_xlim(0, n+2)
    ax.set_ylim(0, n+2)
    ax.set_axis_off()

    def dance_class(offset=0):

        ax.clear()
        ax.set_xlim(0, n + 2)
        ax.set_ylim(0, n + 2)
        ax.set_axis_off()

        count = 0
        np.random.seed(0)
        for y in np.linspace(n + 1, 1, n):
            for x in np.linspace(1, n + 1, n):
                head, chest, arms, hands, joints = get_body_kwargs()

                # q_c = np.random.uniform(0, 2*np.pi)
                q_c = - count * 2 * np.pi / (n ** 2 - 1) + offset
                q_ra = np.random.uniform(0.5, 1.5, 3)
                q_la = np.random.uniform(-0.5, -1.5, 3)
                q = np.hstack((q_c, q_ra, q_la))
                # q[:] = 0
                plot_robot(xy=(x, y), q=q, chest=chest, arms=arms, hands=hands, joints=joints, head=head, ax=ax)
                count += 1

    for i in np.linspace(0, 2*np.pi, 100):
        dance_class(offset=i)
        # plt.pause(0.01)

    # noinspection PyTypeChecker
    ani = FuncAnimation(fig, func=dance_class, frames=np.linspace(0, 4*np.pi, 600), repeat=False)
    ani.save(f'robot_dance{n}x{n}.mp4', writer='ffmpeg', fps=60, metadata=dict(artist='Johannes Tenhumberg'),
             dpi=300)

    # from wzk import save_fig
    # save_fig(filename='robot_dance_class', formats=('png', 'pdf'))


def test2():
    fig, ax = new_fig(aspect=1, width=5, height=5)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

    def ani(q):
        # ax.clear()
        ax.set_xlim(0, 2)
        ax.set_ylim(0, 2)
        ax.set_axis_off()

        head, chest, arms, hands, joints = get_body_kwargs()
        arms['zorder'] = -1
        joints['zorder'] = -1
        hands['zorder'] = -1
        q_ra = [np.sin(q), 2*np.sin(q), 3*np.sin(q)]
        q_la = [np.sin(q), 2*np.sin(q), 3*np.sin(q)]
        # q_ra = np.random.uniform(0.5, 1.5, 3)
        # q_la = np.random.uniform(-0.5, -1.5, 3)

        q = np.hstack((q, q_ra, q_la))
        plot_robot(xy=(1, 1), q=q, chest=chest, arms=arms, hands=hands, joints=joints, head=head, ax=ax)

    # for q_c in np.linspace(0, 2 * np.pi, 100):
    #     ani(q=q_c)
    #     plt.pause(0.01)

    # noinspection PyTypeChecker
    ani = FuncAnimation(fig, func=ani, frames=np.linspace(0, 2 * np.pi, 100), repeat=False)

    uuid = uuid4()
    ani.save(f'robot_dance{uuid}.mp4', writer='ffmpeg', fps=10, metadata=dict(artist='Johannes Tenhumberg'),
             dpi=300)
    save_fig(fig=fig, filename=f"robot_dance{uuid}", formats='png')


if __name__ == '__main__':
    test2()
