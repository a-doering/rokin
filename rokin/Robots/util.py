from rokin.Robots import *  # noqa


def str2robot(robot_id):
    n_dof = int(robot_id[-2:])
    robot_id = robot_id[:-2]
    if robot_id in globals():
        robot = eval("{}(n_dof={})".format(robot_id, n_dof))
    else:
        robot = eval("{}{:0>2}()".format(robot_id, n_dof))

    return robot
