from setuptools import Extension, setup

ext = Extension(
    name='JustinArm07',
    sources=['./topy.cpp', './JustinArm07.cpp'],
    extra_compile_args=['-std=c++1y', '-ffast-math', '-Ofast', '-fpermissive'],
    include_dirs=['/Users/jote/Documents/Code/Software/C/eigen-3.3.7/'],
    library_dirs=[],
    libraries=[],
    language='c++',
)

setup(
    name='JustinArm07',
    version='0.1.0',
    ext_modules=[ext],
)

# Compile via:
# python setup.py develop

