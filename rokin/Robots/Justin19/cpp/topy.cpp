#define PY_SSIZE_T_CLEAN

#include <Python.h>
#include <Eigen/Dense>

#include "Justin19.h"

    
PyObject * get_frames_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * joints;
    int n;
    
    Py_buffer frames_v;
    Py_buffer joints_v;
    static const char * keywords[] = {"frames", "joints", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOi", const_cast<char **>(keywords), &frames, &joints, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    // main
    
    MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
    JOINTS * joints_p = (JOINTS *)joints_v.buf;
    
    FRAMES frames_m(frames_p, N_FRAMES, 1);
    
    for (int i = 0; i < n; ++i) {{
        new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
        get_frames(frames_m, joints_p[i]);
    }}
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&joints_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_dh4_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOi", const_cast<char **>(keywords), &frames, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
    MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
    JOINTS * joints_p = (JOINTS *)joints_v.buf;
    DH4 * dh_p = (DH4 *)dh_v.buf;
    
    FRAMES frames_m(frames_p, N_FRAMES, 1);
    
    for (int i = 0; i < n; ++i) {{
        new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
        get_frames_dh4(frames_m, joints_p[i], *dh_p);
    }}
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_dh4b_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOi", const_cast<char **>(keywords), &frames, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
    MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
    JOINTS * joints_p = (JOINTS *)joints_v.buf;
    DH4 * dh_p = (DH4 *)dh_v.buf;
    
    FRAMES frames_m(frames_p, N_FRAMES, 1);
    
    for (int i = 0; i < n; ++i) {{
        new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
        get_frames_dh4(frames_m, joints_p[i], dh_p[i]);
    }}
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_dh5b_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOi", const_cast<char **>(keywords), &frames, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
    MatrixRow * frames_p = (MatrixRow *)frames_v.buf;
    JOINTS * joints_p = (JOINTS *)joints_v.buf;
    DH5 * dh_p = (DH5 *)dh_v.buf;
    
    FRAMES frames_m(frames_p, N_FRAMES, 1);
    
    for (int i = 0; i < n; ++i) {{
        new (&frames_m) FRAMES(&frames_p[i*N_FRAMES], N_FRAMES, 1);
        get_frames_dh5(frames_m, joints_p[i], dh_p[i]);
    }}
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_jac_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * jacs;
    PyObject * joints;
    int n;
    
    Py_buffer frames_v;
    Py_buffer jacs_v;
    Py_buffer joints_v;
    static const char * keywords[] = {"frames", "jacs", "joints", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOi", const_cast<char **>(keywords), &frames, &jacs, &joints, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(jacs, &jacs_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    // main
    
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
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&jacs_v);
    PyBuffer_Release(&joints_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_jac_dh4_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * jacs;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer jacs_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "jacs", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOOi", const_cast<char **>(keywords), &frames, &jacs, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(jacs, &jacs_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
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
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&jacs_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_jac_dh4b_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * jacs;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer jacs_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "jacs", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOOi", const_cast<char **>(keywords), &frames, &jacs, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(jacs, &jacs_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
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
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&jacs_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}


    
PyObject * get_frames_jac_dh5b_py(PyObject * self, PyObject * args, PyObject * kwargs) {
    
    // initialize_spheres buffer for arguments
    PyObject * frames;
    PyObject * jacs;
    PyObject * joints;
    PyObject * dh;
    int n;
    
    Py_buffer frames_v;
    Py_buffer jacs_v;
    Py_buffer joints_v;
    Py_buffer dh_v;
    static const char * keywords[] = {"frames", "jacs", "joints", "dh", "n", NULL};
    PyArg_ParseTupleAndKeywords(args, kwargs, "OOOOi", const_cast<char **>(keywords), &frames, &jacs, &joints, &dh, &n);
    PyObject_GetBuffer(frames, &frames_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(jacs, &jacs_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(joints, &joints_v, PyBUF_SIMPLE);
    PyObject_GetBuffer(dh, &dh_v, PyBUF_SIMPLE);
    // main
    
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
        
    // release buffer
    PyBuffer_Release(&frames_v);
    PyBuffer_Release(&jacs_v);
    PyBuffer_Release(&joints_v);
    PyBuffer_Release(&dh_v);
    Py_RETURN_NONE;
    
}



    PyMethodDef module_methods[] = {
        {"get_frames", (PyCFunction)get_frames_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_dh4", (PyCFunction)get_frames_dh4_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_dh4b", (PyCFunction)get_frames_dh4b_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_dh5b", (PyCFunction)get_frames_dh5b_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_jacs", (PyCFunction)get_frames_jac_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_jacs_dh4", (PyCFunction)get_frames_jac_dh4_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_jacs_dh4b", (PyCFunction)get_frames_jac_dh4b_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {"get_frames_jacs_dh5b", (PyCFunction)get_frames_jac_dh5b_py, METH_VARARGS | METH_KEYWORDS, NULL},
        {NULL},
    };

    PyModuleDef module_def = {PyModuleDef_HEAD_INIT, "Justin19", NULL, -1, module_methods};

    extern "C" PyObject * PyInit_Justin19() {
        PyObject * module = PyModule_Create(&module_def);
        return module;
    }
    
    