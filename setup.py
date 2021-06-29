import os
import subprocess
from setuptools import setup, find_packages
from setuptools.command.install import install
from setuptools.command.develop import develop


with open("docs/README.md", "r") as fh:
    long_description = fh.read()


def _run(self):
    directory = os.path.split(__file__)[0]
    robot_list = ['Justin19', 'JustinArm07', 'JustinFinger03', 'JustinHand12', 'JustinHand12Cal']
    for robot in robot_list:
        for i in range(10):
            print()
        print(directory)
        print(os.listdir(f'/volume/USERSTORE/tenh_jo/Software/miniconda3/envs/py38test/lib/python3.8/site-packages/rokin/Robots/{robot}/cpp/'))
        # subprocess.call(f"cd {directory}/rokin/Robots/{robot}/cpp; python setup.py develop", shell=True)


class CompileRobotsInstall(install):
    def run(self):
        install.run(self)
        self.announce('INSTALL')
        _run(self)


# class CompileRobotsDevelop(develop):
#     def run(self):
#         develop.run(self)
#         self.announce('DEVELOP')
#         _run(self)


setup(
    name="rokin",
    version="0.0.1",
    author="Johannes Tenhumberg",
    author_email="johannes.tenhumberg@gmail.com",
    description="Robot Kinematic - Python module for robot forward kinematic based on the DH formalism",
    long_description=long_description,
    url="https://github.com/scleronomic/rokin",
    include_package_data=True,
    packages=find_packages(),
    install_requires=['numpy',
                      # 'wzk @ git+https://github.com/scleronomic/WerkZeugKasten'
                      ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],

    cmdclass={'install': CompileRobotsInstall,
              # 'develop': CompileRobotsDevelop
             }
)
