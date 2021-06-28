from setuptools import Extension, setup

import os
eigen_include_dir = os.environ.get("EIGEN_INCLUDE_DIR",                                  # either user defined
                                   os.environ.get("CONDA_PREFIX") + "/include/eigen3/")  # or installed via conda

ext = Extension(
    name='JustinHand12Cal',
    sources=['./topy.cpp', './JustinHand12Cal.cpp'],
    extra_compile_args=['-std=c++1y', '-ffast-math', '-Ofast', '-fpermissive'],
    include_dirs=[eigen_include_dir],
    library_dirs=[],
    libraries=[],
    language='c++',
)

setup(
    name='JustinHand12Cal',
    version='0.1.0',
    ext_modules=[ext],
)

# Compile via:
# python setup.py develop

