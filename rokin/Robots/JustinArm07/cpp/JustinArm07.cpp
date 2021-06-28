
#include <cmath>
#include <Eigen/Dense>

#include "JustinArm07.h"


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
    f(1) << std::cos(q[1]), -std::sin(q[1]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[1]), std::cos(q[1]), 0, 0,
            0, 0, 0, 1;
    f(2) << std::sin(q[2]), std::cos(q[2]), 0, 0,
            0, 0, 1, 0.4,
            std::cos(q[2]), -std::sin(q[2]), 0, 0,
            0, 0, 0, 1;
    f(3) << std::cos(q[3]), -std::sin(q[3]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[3]), std::cos(q[3]), 0, 0,
            0, 0, 0, 1;
    f(4) << -std::cos(q[4]), std::sin(q[4]), 0, 0,
            0, 0, 1, 0.390,
            std::sin(q[4]), std::cos(q[4]), 0, 0,
            0, 0, 0, 1;
    f(5) << -std::sin(q[5]), -std::cos(q[5]), 0, 0,
            0, 0, -1, 0,
            std::cos(q[5]), -std::sin(q[5]), 0, 0,
            0, 0, 0, 1;
    f(6) << std::sin(q[6]), std::cos(q[6]), 0, 0,
            0, 0, -1, 0,
            -std::cos(q[6]), std::sin(q[6]), 0, 0,
            0, 0, 0, 1;
    f(7) << -1., 0, 0, 0,
            0, 0, 1., 0.118,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    f(0) << std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0]), 0, dh[0][2],
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), std::cos(dh[0][3]), dh[0][0]*std::cos(dh[0][3]),
            0, 0, 0, 1;
    f(1) << std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1]), 0, dh[1][2],
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), std::cos(dh[1][3]), dh[1][0]*std::cos(dh[1][3]),
            0, 0, 0, 1;
    f(2) << std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2]), 0, dh[2][2],
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), std::cos(dh[2][3]), dh[2][0]*std::cos(dh[2][3]),
            0, 0, 0, 1;
    f(3) << std::cos(dh[3][1] + q[3]), -std::sin(dh[3][1] + q[3]), 0, dh[3][2],
            std::sin(dh[3][1] + q[3])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[3]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[3]), std::sin(dh[3][3])*std::cos(dh[3][1] + q[3]), std::cos(dh[3][3]), dh[3][0]*std::cos(dh[3][3]),
            0, 0, 0, 1;
    f(4) << std::cos(dh[4][1] + q[4]), -std::sin(dh[4][1] + q[4]), 0, dh[4][2],
            std::sin(dh[4][1] + q[4])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[4]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[4]), std::sin(dh[4][3])*std::cos(dh[4][1] + q[4]), std::cos(dh[4][3]), dh[4][0]*std::cos(dh[4][3]),
            0, 0, 0, 1;
    f(5) << std::cos(dh[5][1] + q[5]), -std::sin(dh[5][1] + q[5]), 0, dh[5][2],
            std::sin(dh[5][1] + q[5])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[5]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[5]), std::sin(dh[5][3])*std::cos(dh[5][1] + q[5]), std::cos(dh[5][3]), dh[5][0]*std::cos(dh[5][3]),
            0, 0, 0, 1;
    f(6) << std::cos(dh[6][1] + q[6]), -std::sin(dh[6][1] + q[6]), 0, dh[6][2],
            std::sin(dh[6][1] + q[6])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[6]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
            std::sin(dh[6][3])*std::sin(dh[6][1] + q[6]), std::sin(dh[6][3])*std::cos(dh[6][1] + q[6]), std::cos(dh[6][3]), dh[6][0]*std::cos(dh[6][3]),
            0, 0, 0, 1;
    f(7) << -1., 0, 0, 0,
            0, 0, 1., 0.118,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    f(0) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) + std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), std::sin(dh[0][4])*std::cos(dh[0][3]), dh[0][2]*std::cos(dh[0][4]) + dh[0][0]*std::sin(dh[0][4])*std::cos(dh[0][3]),
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) - std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), std::cos(dh[0][3])*std::cos(dh[0][4]), -dh[0][2]*std::sin(dh[0][4]) + dh[0][0]*std::cos(dh[0][3])*std::cos(dh[0][4]),
            0, 0, 0, 1;
    f(1) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) + std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), std::sin(dh[1][4])*std::cos(dh[1][3]), dh[1][2]*std::cos(dh[1][4]) + dh[1][0]*std::sin(dh[1][4])*std::cos(dh[1][3]),
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) - std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), std::cos(dh[1][3])*std::cos(dh[1][4]), -dh[1][2]*std::sin(dh[1][4]) + dh[1][0]*std::cos(dh[1][3])*std::cos(dh[1][4]),
            0, 0, 0, 1;
    f(2) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) + std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), std::sin(dh[2][4])*std::cos(dh[2][3]), dh[2][2]*std::cos(dh[2][4]) + dh[2][0]*std::sin(dh[2][4])*std::cos(dh[2][3]),
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) - std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), std::cos(dh[2][3])*std::cos(dh[2][4]), -dh[2][2]*std::sin(dh[2][4]) + dh[2][0]*std::cos(dh[2][3])*std::cos(dh[2][4]),
            0, 0, 0, 1;
    f(3) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[3]) + std::cos(dh[3][4])*std::cos(dh[3][1] + q[3]), std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[3]) - std::sin(dh[3][1] + q[3])*std::cos(dh[3][4]), std::sin(dh[3][4])*std::cos(dh[3][3]), dh[3][2]*std::cos(dh[3][4]) + dh[3][0]*std::sin(dh[3][4])*std::cos(dh[3][3]),
            std::sin(dh[3][1] + q[3])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[3]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[3])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(dh[3][1] + q[3]), std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[3]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[3]), std::cos(dh[3][3])*std::cos(dh[3][4]), -dh[3][2]*std::sin(dh[3][4]) + dh[3][0]*std::cos(dh[3][3])*std::cos(dh[3][4]),
            0, 0, 0, 1;
    f(4) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[4]) + std::cos(dh[4][4])*std::cos(dh[4][1] + q[4]), std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[4]) - std::sin(dh[4][1] + q[4])*std::cos(dh[4][4]), std::sin(dh[4][4])*std::cos(dh[4][3]), dh[4][2]*std::cos(dh[4][4]) + dh[4][0]*std::sin(dh[4][4])*std::cos(dh[4][3]),
            std::sin(dh[4][1] + q[4])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[4]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[4])*std::cos(dh[4][4]) - std::sin(dh[4][4])*std::cos(dh[4][1] + q[4]), std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[4]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[4]), std::cos(dh[4][3])*std::cos(dh[4][4]), -dh[4][2]*std::sin(dh[4][4]) + dh[4][0]*std::cos(dh[4][3])*std::cos(dh[4][4]),
            0, 0, 0, 1;
    f(5) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[5]) + std::cos(dh[5][4])*std::cos(dh[5][1] + q[5]), std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[5]) - std::sin(dh[5][1] + q[5])*std::cos(dh[5][4]), std::sin(dh[5][4])*std::cos(dh[5][3]), dh[5][2]*std::cos(dh[5][4]) + dh[5][0]*std::sin(dh[5][4])*std::cos(dh[5][3]),
            std::sin(dh[5][1] + q[5])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[5]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[5])*std::cos(dh[5][4]) - std::sin(dh[5][4])*std::cos(dh[5][1] + q[5]), std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[5]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[5]), std::cos(dh[5][3])*std::cos(dh[5][4]), -dh[5][2]*std::sin(dh[5][4]) + dh[5][0]*std::cos(dh[5][3])*std::cos(dh[5][4]),
            0, 0, 0, 1;
    f(6) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[6]) + std::cos(dh[6][4])*std::cos(dh[6][1] + q[6]), std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[6]) - std::sin(dh[6][1] + q[6])*std::cos(dh[6][4]), std::sin(dh[6][4])*std::cos(dh[6][3]), dh[6][2]*std::cos(dh[6][4]) + dh[6][0]*std::sin(dh[6][4])*std::cos(dh[6][3]),
            std::sin(dh[6][1] + q[6])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[6]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
            std::sin(dh[6][3])*std::sin(dh[6][1] + q[6])*std::cos(dh[6][4]) - std::sin(dh[6][4])*std::cos(dh[6][1] + q[6]), std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[6]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[6]), std::cos(dh[6][3])*std::cos(dh[6][4]), -dh[6][2]*std::sin(dh[6][4]) + dh[6][0]*std::cos(dh[6][3])*std::cos(dh[6][4]),
            0, 0, 0, 1;
    f(7) << -1., 0, 0, 0,
            0, 0, 1., 0.118,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void _fill_jac(JACS& j, JOINTS& q){
    j(0, 0) << -std::sin(q[0]), -std::cos(q[0]), 0, 0,
               std::cos(q[0]), -std::sin(q[0]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(1, 1) << -std::sin(q[1]), -std::cos(q[1]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[1]), -std::sin(q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 2) << std::cos(q[2]), -std::sin(q[2]), 0, 0,
               0, 0, 0, 0,
               -std::sin(q[2]), -std::cos(q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 3) << -std::sin(q[3]), -std::cos(q[3]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[3]), -std::sin(q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 4) << std::sin(q[4]), std::cos(q[4]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[4]), -std::sin(q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 5) << -std::cos(q[5]), std::sin(q[5]), 0, 0,
               0, 0, 0, 0,
               -std::sin(q[5]), -std::cos(q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 6) << std::cos(q[6]), -std::sin(q[6]), 0, 0,
               0, 0, 0, 0,
               std::sin(q[6]), std::cos(q[6]), 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh){
    j(0, 0) << -std::sin(dh[0][1] + q[0]), -std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 1) << -std::sin(dh[1][1] + q[1]), -std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 2) << -std::sin(dh[2][1] + q[2]), -std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 3) << -std::sin(dh[3][1] + q[3]), -std::cos(dh[3][1] + q[3]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[3]), -std::sin(dh[3][1] + q[3])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][1] + q[3]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 4) << -std::sin(dh[4][1] + q[4]), -std::cos(dh[4][1] + q[4]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[4]), -std::sin(dh[4][1] + q[4])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][1] + q[4]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 5) << -std::sin(dh[5][1] + q[5]), -std::cos(dh[5][1] + q[5]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[5]), -std::sin(dh[5][1] + q[5])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][1] + q[5]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 6) << -std::sin(dh[6][1] + q[6]), -std::cos(dh[6][1] + q[6]), 0, 0,
               std::cos(dh[6][3])*std::cos(dh[6][1] + q[6]), -std::sin(dh[6][1] + q[6])*std::cos(dh[6][3]), 0, 0,
               std::sin(dh[6][3])*std::cos(dh[6][1] + q[6]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[6]), 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh){
    j(0, 0) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), -std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) - std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) + std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 1) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), -std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) - std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) + std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 2) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), -std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) - std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) + std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 3) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[3]) - std::sin(dh[3][1] + q[3])*std::cos(dh[3][4]), -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[3]) - std::cos(dh[3][4])*std::cos(dh[3][1] + q[3]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[3]), -std::sin(dh[3][1] + q[3])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[3]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[3]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[3])*std::cos(dh[3][4]) + std::sin(dh[3][4])*std::cos(dh[3][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 4) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[4]) - std::sin(dh[4][1] + q[4])*std::cos(dh[4][4]), -std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[4]) - std::cos(dh[4][4])*std::cos(dh[4][1] + q[4]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[4]), -std::sin(dh[4][1] + q[4])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[4]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[4]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[4])*std::cos(dh[4][4]) + std::sin(dh[4][4])*std::cos(dh[4][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 5) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[5]) - std::sin(dh[5][1] + q[5])*std::cos(dh[5][4]), -std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[5]) - std::cos(dh[5][4])*std::cos(dh[5][1] + q[5]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[5]), -std::sin(dh[5][1] + q[5])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[5]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[5]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[5])*std::cos(dh[5][4]) + std::sin(dh[5][4])*std::cos(dh[5][1] + q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 6) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[6]) - std::sin(dh[6][1] + q[6])*std::cos(dh[6][4]), -std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[6]) - std::cos(dh[6][4])*std::cos(dh[6][1] + q[6]), 0, 0,
               std::cos(dh[6][3])*std::cos(dh[6][1] + q[6]), -std::sin(dh[6][1] + q[6])*std::cos(dh[6][3]), 0, 0,
               std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[6]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[6]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[6])*std::cos(dh[6][4]) + std::sin(dh[6][4])*std::cos(dh[6][1] + q[6]), 0, 0,
               0, 0, 0, 0;
    
}


void combine_frames(FRAMES& f){
    f(1) = f(0) * f(1);
    f(2) = f(1) * f(2);
    f(3) = f(2) * f(3);
    f(4) = f(3) * f(4);
    f(5) = f(4) * f(5);
    f(6) = f(5) * f(6);
    f(7) = f(6) * f(7);
    
}


void combine_jacs(JACS& j, DICT& d){
    // 0
    j(0, 1) = j(0, 0) * d(1, 1);
    j(0, 2) = j(0, 0) * d(1, 2);
    j(0, 3) = j(0, 0) * d(1, 3);
    j(0, 4) = j(0, 0) * d(1, 4);
    j(0, 5) = j(0, 0) * d(1, 5);
    j(0, 6) = j(0, 0) * d(1, 6);
    j(0, 7) = j(0, 0) * d(1, 7);
    // 1
    j(1, 1) = d(0, 0) * j(1, 1);
    j(1, 2) = j(1, 1) * d(2, 2);
    j(1, 3) = j(1, 1) * d(2, 3);
    j(1, 4) = j(1, 1) * d(2, 4);
    j(1, 5) = j(1, 1) * d(2, 5);
    j(1, 6) = j(1, 1) * d(2, 6);
    j(1, 7) = j(1, 1) * d(2, 7);
    // 2
    j(2, 2) = d(0, 1) * j(2, 2);
    j(2, 3) = j(2, 2) * d(3, 3);
    j(2, 4) = j(2, 2) * d(3, 4);
    j(2, 5) = j(2, 2) * d(3, 5);
    j(2, 6) = j(2, 2) * d(3, 6);
    j(2, 7) = j(2, 2) * d(3, 7);
    // 3
    j(3, 3) = d(0, 2) * j(3, 3);
    j(3, 4) = j(3, 3) * d(4, 4);
    j(3, 5) = j(3, 3) * d(4, 5);
    j(3, 6) = j(3, 3) * d(4, 6);
    j(3, 7) = j(3, 3) * d(4, 7);
    // 4
    j(4, 4) = d(0, 3) * j(4, 4);
    j(4, 5) = j(4, 4) * d(5, 5);
    j(4, 6) = j(4, 4) * d(5, 6);
    j(4, 7) = j(4, 4) * d(5, 7);
    // 5
    j(5, 5) = d(0, 4) * j(5, 5);
    j(5, 6) = j(5, 5) * d(6, 6);
    j(5, 7) = j(5, 5) * d(6, 7);
    // 6
    j(6, 6) = d(0, 5) * j(6, 6);
    j(6, 7) = j(6, 6) * d(7, 7);
    
}


void combine_dict(FRAMES& f, DICT& d){
    // 7
    d(7, 7) = f(7);
    // 6
    d(6, 6) = f(6);
    d(6, 7) = f(6) * d(7, 7);
    // 5
    d(5, 5) = f(5);
    d(5, 6) = f(5) * d(6, 6);
    d(5, 7) = f(5) * d(6, 7);
    // 4
    d(4, 4) = f(4);
    d(4, 5) = f(4) * d(5, 5);
    d(4, 6) = f(4) * d(5, 6);
    d(4, 7) = f(4) * d(5, 7);
    // 3
    d(3, 3) = f(3);
    d(3, 4) = f(3) * d(4, 4);
    d(3, 5) = f(3) * d(4, 5);
    d(3, 6) = f(3) * d(4, 6);
    d(3, 7) = f(3) * d(4, 7);
    // 2
    d(2, 2) = f(2);
    d(2, 3) = f(2) * d(3, 3);
    d(2, 4) = f(2) * d(3, 4);
    d(2, 5) = f(2) * d(3, 5);
    d(2, 6) = f(2) * d(3, 6);
    d(2, 7) = f(2) * d(3, 7);
    // 1
    d(1, 1) = f(1);
    d(1, 2) = f(1) * d(2, 2);
    d(1, 3) = f(1) * d(2, 3);
    d(1, 4) = f(1) * d(2, 4);
    d(1, 5) = f(1) * d(2, 5);
    d(1, 6) = f(1) * d(2, 6);
    d(1, 7) = f(1) * d(2, 7);
    // 0
    d(0, 0) = f(0);
    d(0, 1) = f(0) * d(1, 1);
    d(0, 2) = f(0) * d(1, 2);
    d(0, 3) = f(0) * d(1, 3);
    d(0, 4) = f(0) * d(1, 4);
    d(0, 5) = f(0) * d(1, 5);
    d(0, 6) = f(0) * d(1, 6);
    d(0, 7) = f(0) * d(1, 7);
    
}

    