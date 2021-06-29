import os
import subprocess
from setuptools import setup, find_packages
from setuptools.command.install import install
from setuptools.command.develop import develop
from setuptools.command.build_py import build_py


with open("docs/README.md", "r") as fh:
    long_description = fh.read()

directory = os.path.split(__file__)[0]


def _run(self):
    b = a
    print(directory)
    # subprocess.call(f"cd {directory}/rokin/Robots/Justin19/cpp; python setup.py develop", shell=True)
    # subprocess.call(f"cd {directory}/rokin/Robots/JustinArm07/cpp; python setup.py develop", shell=True)
    # subprocess.call(f"cd {directory}/rokin/Robots/JustinFinger03/cpp; python setup.py develop", shell=True)
    # subprocess.call(f"cd {directory}/rokin/Robots/JustinHand12/cpp; python setup.py develop", shell=True)
    # subprocess.call(f"cd {directory}/rokin/Robots/JustinHand12Cal/cpp; python setup.py develop", shell=True)
    self.announce('COMPILED ROBOTS')
    self.announce('COMPILED ROBOTS')
    self.announce('COMPILED ROBOTS')
    self.announce('COMPILED ROBOTS')


class CompileRobotsInstall(install):
    def run(self):
        install.run(self)
        _run(self)


class CompileRobotsDevelop(develop):
    def run(self):
        develop.run(self)
        _run(self)


class CompileRobotsBuildPy(build_py):
    def run(self):
        build_py.run(self)
        _run(self)



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
                      #'wzk @ git+https://github.com/scleronomic/WerkZeugKasten'
                      ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],

    cmdclass={'install': CompileRobotsInstall,
              'develop': CompileRobotsDevelop,
              'build_py': CompileRobotsBuildPy,
              }
)
