import os
import subprocess
from wzk import safe_create_dir, str2file

from rokin.CodeGeneration import symbolic
from rokin.CodeGeneration import code_generation, code_stubs


def get_directory(robot):
    directory = os.path.split(__file__)[0] + f'/../Robots/{robot.id}/cpp'
    directory = os.path.normpath(directory)
    return directory


def check_old_files(directory, files):
    ld = os.listdir(directory)
    old = True
    for f in files:
        if f not in ld:
            old = False
    return old


def assert_robot(robot):
    assert robot.n_frames == len(robot.next_frame_idx)
    assert robot.n_frames == len(robot.prev_frame_idx)
    # TODO add more sanity checks


def generate_robot_cpp(robot, dh_mode='fix', replace=True,
                       verbose=1):

    directory = get_directory(robot=robot)
    safe_create_dir(directory=directory)

    if not replace and check_old_files(directory=directory, files=[f"{robot.id}.cpp", f"{robot.id}.h"]):
        if verbose > 0:
            print(f"Abort code generation:\n "
                  f"Files for the robot {robot.id} in {directory} exist already."
                  f"Use replace=True to overwrite the files.")
        return

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

    str2file(directory=directory, **{'__init__.py': '',
                                     'setup.py': setup_py,
                                     'topy.cpp': topy_cpp,
                                     f'{robot.id}.h': robot_h,
                                     f'{robot.id}.cpp': robot_cpp})
    if verbose > 0:
        print(f"Successfully generated code for the robot {_robot.id} in the directory {directory}")


def compile_robot_cpp(robot, replace=False, verbose=0):
    directory = get_directory(robot=robot)
    if not replace and check_old_files(directory=directory, files=[f"{robot.id}.egg-info", "build"]):
        if verbose > 0:
            print(f"Abort code compiling:\n "
                  f"Files for the robot {robot.id} in {directory} exist already."
                  f"Use replace=True to overwrite the files.")
        return

    quite = f" > {os.devnull}" if verbose < 2 else ""
    subprocess.call(f"cd {directory}; python setup.py develop{quite}", shell=True)
    if verbose > 0:
        print(f"Successfully compiled code for the robot {robot.id} in the directory {directory}.")


if __name__ == '__main__':
    from rokin.Robots import JustinHand12
    _robot = JustinHand12()
    generate_robot_cpp(robot=_robot)
