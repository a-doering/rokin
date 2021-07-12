from rokin.Robots import *  # noqa


def str2robot(robot_id):
    n_dof = int(robot_id[-2:])
    robot_id = robot_id[:-2]
    if robot_id in globals():
        robot = eval("{}(n_dof={})".format(robot_id, n_dof))
    else:
        robot = eval("{}{:0>2}()".format(robot_id, n_dof))

    return robot


def _generate_and_compile_all():
    robot_list = [JustinFinger03, JustinArm07, JustinHand12, Justin19]
    print(f'Generate and compile the C++ code for {[robot.id for robot in robot_list]}, this may take a few minutes...')
    for robot in robot_list:
        import_robot_cpp(robot=robot, warning=False)
