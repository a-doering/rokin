from rokin.Robots import *  # noqa


def str2robot(robot_id):
    n_dof = int(robot_id[-2:])
    robot_id = robot_id[:-2]
    if robot_id in globals():
        robot = eval("{}(n_dof={})".format(robot_id, n_dof))
    else:
        robot = eval("{}{:0>2}()".format(robot_id, n_dof))

    return robot


# robot_ids = ['SingleSphere02', 'SingleSphere03',
#              'StaticArm02', 'StaticArm07',
#              'MovingArm01', 'MovingArm06',
#              'Justin19', 'JustinArm07',
#              'JustinBase03', 'JustinBase05',
#              'JustinFinger03',
#              'JustinHand12']
#
# for ro in robot_ids:
#     print(ro)
#     str2robot(ro)
