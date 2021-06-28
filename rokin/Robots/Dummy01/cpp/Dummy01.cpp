
#include <cmath>
#include <Eigen/Dense>

#include "Dummy01.h"


void get_frames(FRAMES& f, JOINTS& q){
    fill_frames(f, q);
    combine_frames(f);
}


void get_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    fill_frames_dh4(f, q, dh);
    combine_frames(f);
}


void get_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    fill_frames_dh5(f, q, dh);
    combine_frames(f);
}


void get_frames_jacs(FRAMES& f, JACS& j, JOINTS& q){

    fill_frames(f, q);
    _fill_jac(j, q);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}


void get_frames_jacs_dh4(FRAMES& f, JACS& j, JOINTS& q, DH4& dh){

    fill_frames_dh4(f, q, dh);
    fill_jacs_dh4(j, q, dh);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}


void get_frames_jacs_dh5(FRAMES& f, JACS& j, JOINTS& q, DH5& dh){

    fill_frames_dh5(f, q, dh);
    fill_jacs_dh5(j, q, dh);

    DICT d;
    combine_dict(f, d);
    combine_jacs(j, d);
    f = d.row(0);
}


inline void fill_frames(FRAMES& f, JOINTS& q){
    f(0) << std::cos(q[0]), -std::sin(q[0]), 0, 0,
            std::sin(q[0]), std::cos(q[0]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    f(0) << std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0]), 0, dh[0][2],
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), std::cos(dh[0][3]), dh[0][0]*std::cos(dh[0][3]),
            0, 0, 0, 1;
    
}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    f(0) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) + std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), std::sin(dh[0][4])*std::cos(dh[0][3]), dh[0][2]*std::cos(dh[0][4]) + dh[0][0]*std::sin(dh[0][4])*std::cos(dh[0][3]),
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) - std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), std::cos(dh[0][3])*std::cos(dh[0][4]), -dh[0][2]*std::sin(dh[0][4]) + dh[0][0]*std::cos(dh[0][3])*std::cos(dh[0][4]),
            0, 0, 0, 1;
    
}


inline void _fill_jac(JACS& j, JOINTS& q){
    j(0, 0) << -std::sin(q[0]), -std::cos(q[0]), 0, 0,
               std::cos(q[0]), -std::sin(q[0]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh){
    j(0, 0) << -std::sin(dh[0][1] + q[0]), -std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh){
    j(0, 0) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), -std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) - std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) + std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    
}


void combine_frames(FRAMES& f){
    
}


void combine_jacs(JACS& j, DICT& d){
    // 0
    
}


void combine_dict(FRAMES& f, DICT& d){
    // 0
    d(0, 0) = f(0);
    
}

    