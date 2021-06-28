from wzk.strings import tab_str


__numeric_type = 'double'
# Finding float was not faster than double for Justin19
#   but when calculating the numeric derivative you get inaccuracies when switching the data types, so double wins


def __topy_function_stub(fun_name, var_names, var_types, code):
    # see also https://docs.python.org/3/c-api/arg.html for type_dict
    __nl = '\n'
    type_dict = dict(i='int', f='float', d='double', O='PyObject *')
    vars_a = ', '.join([f'"{v}"' for v in var_names])               # apostrophe
    vars_r = ', '.join([f'&{v}' for v in var_names])                # reference
    vars_o = [n for n, t in zip(var_names, var_types) if t == 'O']  # object

    s = f"""
PyObject * {fun_name}(PyObject * self, PyObject * args, PyObject * kwargs) {{
{tab_str(f'''
// initialize_spheres buffer for arguments
{__nl.join([f'{type_dict[t]} {n};' for n, t in zip(var_names, var_types)])}

{__nl.join([f'Py_buffer {n}_v;' for n in vars_o])}
static const char * keywords[] = {{{vars_a}, NULL}};
PyArg_ParseTupleAndKeywords(args, kwargs, "{var_types}", const_cast<char **>(keywords), {vars_r});
{__nl.join([f'PyObject_GetBuffer({n}, &{n}_v, PyBUF_SIMPLE);' for n in vars_o])}
// main
{code}
// release buffer
{__nl.join([f'PyBuffer_Release(&{n}_v);' for n in vars_o])}
Py_RETURN_NONE;
''')}
}}
"""
    return s


def get_setup_py(robot_id, eigen):
    # TODO '-std=c++1y' <-> '-std=c++14'
    setup_py = f"""from setuptools import Extension, setup

ext = Extension(
    name='{robot_id}',
    sources=['./topy.cpp', './{robot_id}.cpp'],
    extra_compile_args=['-std=c++1y', '-ffast-math', '-Ofast', '-fpermissive'],
    include_dirs=['{eigen}'],
    library_dirs=[],
    libraries=[],
    language='c++',
)

setup(
    name='{robot_id}',
    version='0.1.0',
    ext_modules=[ext],
)

# Compile via:
# python setup.py develop

"""
    return setup_py


def get_topy_cpp(robot_id):

    topy_cpp = f"""#define PY_SSIZE_T_CLEAN

#include <Python.h>
#include <Eigen/Dense>

#include "{robot_id}.h"

    {__topy_function_stub(fun_name='get_frames_py', 
                          var_names=['frames', 'joints', 'n'], 
                          var_types='OOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    get_frames(frames_m, joints_p[i]);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_dh4_py', 
                          var_names=['frames', 'joints', 'dh', 'n'], 
                          var_types='OOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH4 * dh_p = (DH4 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    get_frames_dh4(frames_m, joints_p[i], *dh_p);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_dh4b_py', 
                          var_names=['frames', 'joints', 'dh', 'n'], 
                          var_types='OOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH4 * dh_p = (DH4 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    get_frames_dh4(frames_m, joints_p[i], dh_p[i]);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_dh5b_py', 
                          var_names=['frames', 'joints', 'dh', 'n'], 
                          var_types='OOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH5 * dh_p = (DH5 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    get_frames_dh5(frames_m, joints_p[i], dh_p[i]);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_jac_py', 
                          var_names=['frames', 'jacs', 'joints', 'n'], 
                          var_types='OOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
MatrixRow * jacs_p = (MatrixRow *)jacs_v.buf;
JACS jacs_m(jacs_p, N_JOINTS, N_FRAMES);
JOINTS * joints_p = (JOINTS *)joints_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    new (&jacs_m) JACS(&jacs_p[i*N_JOINTS*N_FRAMES], N_JOINTS, N_FRAMES);
    get_frames_jacs(frames_m, jacs_m, joints_p[i]);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_jac_dh4_py', 
                          var_names=['frames', 'jacs', 'joints', 'dh', 'n'], 
                          var_types='OOOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
MatrixRow * jacs_p = (MatrixRow *)jacs_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH4 * dh_p = (DH4 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);
JACS jacs_m(jacs_p, N_JOINTS, N_FRAMES);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    new (&jacs_m) JACS(&jacs_p[i*N_JOINTS*N_FRAMES], N_JOINTS, N_FRAMES);
    get_frames_jacs_dh4(frames_m, jacs_m, joints_p[i], *dh_p);
    }}
    ''')}

    {__topy_function_stub(fun_name='get_frames_jac_dh4b_py', 
                          var_names=['frames', 'jacs', 'joints', 'dh', 'n'], 
                          var_types='OOOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
MatrixRow * jacs_p = (MatrixRow *)jacs_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH4 * dh_p = (DH4 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);
JACS jacs_m(jacs_p, N_JOINTS, N_FRAMES);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    new (&jacs_m) JACS(&jacs_p[i*N_JOINTS*N_FRAMES], N_JOINTS, N_FRAMES);
    get_frames_jacs_dh4(frames_m, jacs_m, joints_p[i], dh_p[i]);
}}
    ''')}

    {__topy_function_stub(fun_name='get_frames_jac_dh5b_py', 
                          var_names=['frames', 'jacs', 'joints', 'dh', 'n'], 
                          var_types='OOOOi',
                          code='''
MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
MatrixRow * jacs_p = (MatrixRow *)jacs_v.buf;
JOINTS * joints_p = (JOINTS *)joints_v.buf;
DH5 * dh_p = (DH5 *)dh_v.buf;

FRAMES frames_m(frames_p, N_FRAMES, 1);
JACS jacs_m(jacs_p, N_JOINTS, N_FRAMES);

for (int i = 0; i < n; ++i) {{
    new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
    new (&jacs_m) JACS(&jacs_p[i*N_JOINTS*N_FRAMES], N_JOINTS, N_FRAMES);
    get_frames_jacs_dh5(frames_m, jacs_m, joints_p[i], dh_p[i]);
}}
    ''')}


    PyMethodDef module_methods[] = {{
        {{"get_frames", (PyCFunction)get_frames_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_dh4", (PyCFunction)get_frames_dh4_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_dh4b", (PyCFunction)get_frames_dh4b_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_dh5b", (PyCFunction)get_frames_dh5b_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_jacs", (PyCFunction)get_frames_jac_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_jacs_dh4", (PyCFunction)get_frames_jac_dh4_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_jacs_dh4b", (PyCFunction)get_frames_jac_dh4b_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{"get_frames_jacs_dh5b", (PyCFunction)get_frames_jac_dh5b_py, METH_VARARGS | METH_KEYWORDS, NULL}},
        {{NULL}},
    }};

    PyModuleDef module_def = {{PyModuleDef_HEAD_INIT, "{robot_id}", NULL, -1, module_methods}};

    extern "C" PyObject * PyInit_{robot_id}() {{
        PyObject * module = PyModule_Create(&module_def);
        return module;
    }}
    
    """

    return topy_cpp


def get_robot_h(robot):
    robot_h = f"""#ifndef {robot.id.upper()}_H
#define {robot.id.upper()}_H

#include <cmath>
#include <Eigen/Dense>

const double pi = 3.141592653589793;

const int N_JOINTS = {robot.n_dof};
const int N_FRAMES = {robot.n_frames};
const int N_DH = {len(robot.dh)};

typedef Eigen::Matrix<{__numeric_type}, {robot.n_dim + 1}, {robot.n_dim + 1}, Eigen::RowMajor> MatrixRow;
typedef Eigen::Map<Eigen::Array<MatrixRow, N_FRAMES, 1, Eigen::ColMajor> > FRAMES;
typedef Eigen::Map<Eigen::Array<MatrixRow,  N_JOINTS, N_FRAMES, Eigen::RowMajor> > JACS;
typedef Eigen::Array<MatrixRow, N_FRAMES, N_FRAMES, Eigen::RowMajor> DICT;
typedef {__numeric_type} JOINTS[N_JOINTS];
typedef {__numeric_type} DH4[N_DH][4];
typedef {__numeric_type} DH5[N_DH][5];

void get_frames(FRAMES& frames, JOINTS& q);
void get_frames_dh4(FRAMES& frames, JOINTS& q, DH4& dh);
void get_frames_dh5(FRAMES& frames, JOINTS& q, DH5& dh);
void get_frames_jacs(FRAMES& f, JACS& j, JOINTS& q);
void get_frames_jacs_dh4(FRAMES& f, JACS& j, JOINTS& q, DH4& dh);
void get_frames_jacs_dh5(FRAMES& f, JACS& j, JOINTS& q, DH5& dh);

inline void fill_frames(FRAMES& f, JOINTS& q);
inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh);
inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh);
inline void _fill_jac(JACS& j, JOINTS& q);
inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh);
inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh);

void combine_frames(FRAMES& f);
void combine_dict(FRAMES& f, DICT& d);
void combine_jacs(JACS& j, DICT& d);

#endif // {robot.id.upper()}_H

    """

    return robot_h


def get_robot_cpp(robot,
                  s_fj, s_combine_f, s_combine_d, s_combine_j):

    s_fj = tuple((tab_str(*_s_fj, tab=4) for _s_fj in s_fj))
    s_combine_f, s_combine_d, s_combine_j = tab_str(s_combine_f, s_combine_d, s_combine_j, tab=4)

    robot_cpp = f"""#include "{robot.id}.h"


void get_frames(FRAMES& f, JOINTS& q){{
    fill_frames(f, q);
    combine_frames(f);
}}


void get_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){{
    fill_frames_dh4(f, q, dh);
    combine_frames(f);
}}


void get_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){{
    fill_frames_dh5(f, q, dh);
    combine_frames(f);
}}


void get_frames_jacs(FRAMES& f, JACS& j, JOINTS& q){{

    fill_frames(f, q);
    _fill_jac(j, q);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}}


void get_frames_jacs_dh4(FRAMES& f, JACS& j, JOINTS& q, DH4& dh){{

    fill_frames_dh4(f, q, dh);
    fill_jacs_dh4(j, q, dh);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}}


void get_frames_jacs_dh5(FRAMES& f, JACS& j, JOINTS& q, DH5& dh){{

    fill_frames_dh5(f, q, dh);
    fill_jacs_dh5(j, q, dh);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}}


inline void fill_frames(FRAMES& f, JOINTS& q){{
{s_fj[0][0]}
}}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){{
{s_fj[1][0]}
}}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){{
{s_fj[2][0]}
}}


inline void _fill_jac(JACS& j, JOINTS& q){{
{s_fj[0][1]}
}}


inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh){{
{s_fj[1][1]}
}}


inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh){{
{s_fj[2][1]}
}}


void combine_frames(FRAMES& f){{
{s_combine_f}
}}


void combine_jacs(JACS& j, DICT& d){{
{s_combine_j}
}}


void combine_dict(FRAMES& f, DICT& d){{
{s_combine_d}
}}

    """
    return robot_cpp
