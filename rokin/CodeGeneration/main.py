import os
from wzk import safe_create_dir, str2file

from rokin.CodeGeneration import symbolic
from rokin.CodeGeneration import code_generation, code_stubs


def main(robot, dh_mode='fix'):

    assert robot.n_frames == len(robot.next_frame_idx)
    assert robot.n_frames == len(robot.prev_frame_idx)

    modes = ['fix', 'var4', 'var5']
    frames_jacs = [symbolic.get_frames_jacs(robot=robot, dh_mode=m) for m in modes]

    s_fj = [(code_generation.fill_frames_str(frames=f), code_generation.fill_jacs_str(frames_jac=j))
            for f, j in frames_jacs]

    s_combine_f = code_generation.combine_frames_str(nfi=robot.next_frame_idx)
    s_create_d = code_generation.combine_dict_str(nfi=robot.next_frame_idx)
    s_combine_j = code_generation.combine_jacs_str(nfi=robot.next_frame_idx, joint_frame_idx=robot.joint_frame_idx)

    setup_py = code_stubs.get_setup_py(robot_id=robot.id)
    topy_cpp = code_stubs.get_topy_cpp(robot_id=robot.id)
    robot_h = code_stubs.get_robot_h(robot=robot)
    robot_cpp = code_stubs.get_robot_cpp(robot=robot, s_fj=s_fj,
                                         s_combine_f=s_combine_f, s_combine_j=s_combine_j, s_combine_d=s_create_d)

    safe_create_dir(directory=directory)
    str2file(directory=directory, **{'setup.py': setup_py,
                                     'topy.cpp': topy_cpp,
                                     f'{robot.id}.h': robot_h,
                                     f'{robot.id}.cpp': robot_cpp})


if __name__ == '__main__':
    from rokin.Robots import JustinHand12
    _robot = JustinHand12()

    directory = os.path.split(__file__)[0] + f'/../Robots/{_robot.id}/cpp'
    directory = os.path.normpath(directory)

    main(robot=_robot)
    print(f"Successfully generated C++ code for the robot {_robot.id} in the directory {directory}")
