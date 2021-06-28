from setuptools import Extension, setup

ext = Extension(
    name='Dummy01',
    sources=['./topy.cpp', './Dummy01.cpp'],
    extra_compile_args=['-std=c++1y', '-ffast-math', '-Ofast', '-fpermissive'],
    include_dirs=['/Users/jote/Documents/Code/Software/C/eigen-3.3.7/'],
    library_dirs=[],
    libraries=[],
    language='c++',
)

setup(
    name='Dummy01',
    version='0.1.0',
    ext_modules=[ext],
)

# Compile via:
# python setup.py develop

