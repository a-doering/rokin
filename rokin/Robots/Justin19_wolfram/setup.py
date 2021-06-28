import numpy
from distutils.core import setup
from distutils.extension import Extension
import Cython.Compiler.Options
from Cython.Distutils import build_ext
Cython.Compiler.Options.annotate = True


ext_modules = [Extension(
    name='Justin19_wolfram',
    sources=['Kinematic/Robots/Justin_old/justin_cython/justin19_.pyx'],
    include_dirs=[numpy.get_include(),
                  '/home/baeuml/src/ajustin/pkgs/ajustin/',
                  '/home/baeuml/src/ajustin/pkgs/ajustin/build/release/sled11-x86-64/'],
    library_dirs=['/home/baeuml/src/ajustin/pkgs/ajustin/build/release/sled11-x86-64/lib/'],
    libraries=['crobotkinematics-wrapper'],  # http://nealhughes.net/cython1/
    extra_compile_args=['-std=c++14', '-ffast-math', '-Ofast'],
    # extra_objects=["fc.c"],  # if you compile fc.bullet separately
    language='c++',
    # extra_link_args = "...".split()
    )]

setup(
    name='Justin19_wolfram',
    cmdclass={'build_ext': build_ext},
    ext_modules=ext_modules,
    # ext_modules = cythonize(ext_modules)  ? not in 0.14.1
    # version=
    # description=
    # author=
    # author_email=
    )

# Make sure the library of Berthold Baeuml is added to $LD_LIBRARY_PATH
# echo $LD_LIBRARY_PATH
# export LD_LIBRARY_PATH="/home/baeuml/src/ajustin/pkgs/ajustin/build/release/sled11-x86-64/lib/:$LD_LIBRARY_PATH"


# Test if everything works correct by compiling a test file and running it should take < 50 microseconds (1e-6 s)
# gcc -std=c99 -c kin-test ForwardKinematic/kin_test.c -I /home/baeuml/src/osl42/ajustin/pkgs/ajustin/
#   -L /home/baeuml/src/osl42/ajustin/pkgs/ajustin/build/release/sled11-x86-64/lib/ -l crobotkinematics-wrapper
# kin-test


# RUN a_cython_setup.py to create the .c and the .so file in the desired directory
# python Kinematic/Robots/Justin_old/justin_cython/setup.py build_ext
#   --build-lib "Kinematic/Robots/Justin_old/justin_cython/" --verbose
