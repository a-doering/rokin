import os
import subprocess
from setuptools import setup, find_packages
from setuptools.command.install import install

with open("docs/README.md", "r") as fh:
    long_description = fh.read()


directory = os.path.split(__file__)[0]


class InstallLocalPackage(install):
    def run(self):
        install.run(self)
        print(directory)
        subprocess.call(f"cd {directory}/rokin/Robots/JustinFinger03/cpp; python setup.py develop", shell=True)


setup(
    name="rokin",
    version="0.0.1",
    author="Johannes Tenhumberg",
    author_email="johannes.tenhumberg@gmail.com",
    description="Robot Kinematic - Python module for robot forward kinematic based on the DH formalism",
    long_description=long_description,
    url="https://github.com/scleronomic/rokin",
    packages=find_packages(),
    install_requires=['numpy',
                      'wzk @ git+https://github.com/scleronomic/WerkZeugKasten'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    cmdclass={'install': InstallLocalPackage}

)
