#include "JustinFinger03.h"


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
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0,
            0, 0, 0, 1.;
    f(1) << std::cos(q[0]), -std::sin(q[0]), 0, 0,
            std::sin(q[0]), std::cos(q[0]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(2) << std::cos(q[1]), -std::sin(q[1]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[1]), std::cos(q[1]), 0, 0,
            0, 0, 0, 1;
    f(3) << std::cos(q[2]), -std::sin(q[2]), 0, 0.0750,
            std::sin(q[2]), std::cos(q[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(4) << std::cos(q[2] - 0.5*pi), std::cos(q[2]), 0, 0.04,
            std::sin(q[2] - 0.5*pi), std::cos(q[2] - 0.5*pi), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(5) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0,
            0, 0, 0, 1.;
    f(1) << std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0]), 0, dh[0][2],
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), std::cos(dh[0][3]), dh[0][0]*std::cos(dh[0][3]),
            0, 0, 0, 1;
    f(2) << std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1]), 0, dh[1][2],
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), std::cos(dh[1][3]), dh[1][0]*std::cos(dh[1][3]),
            0, 0, 0, 1;
    f(3) << std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2]), 0, dh[2][2],
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), std::cos(dh[2][3]), dh[2][0]*std::cos(dh[2][3]),
            0, 0, 0, 1;
    f(4) << std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2]), 0, dh[3][2],
            std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[2]), std::sin(dh[3][3])*std::cos(dh[3][1] + q[2]), std::cos(dh[3][3]), dh[3][0]*std::cos(dh[3][3]),
            0, 0, 0, 1;
    f(5) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0,
            0, 0, 0, 1.;
    f(1) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) + std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), std::sin(dh[0][4])*std::cos(dh[0][3]), dh[0][2]*std::cos(dh[0][4]) + dh[0][0]*std::sin(dh[0][4])*std::cos(dh[0][3]),
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) - std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), std::cos(dh[0][3])*std::cos(dh[0][4]), -dh[0][2]*std::sin(dh[0][4]) + dh[0][0]*std::cos(dh[0][3])*std::cos(dh[0][4]),
            0, 0, 0, 1;
    f(2) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) + std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), std::sin(dh[1][4])*std::cos(dh[1][3]), dh[1][2]*std::cos(dh[1][4]) + dh[1][0]*std::sin(dh[1][4])*std::cos(dh[1][3]),
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) - std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), std::cos(dh[1][3])*std::cos(dh[1][4]), -dh[1][2]*std::sin(dh[1][4]) + dh[1][0]*std::cos(dh[1][3])*std::cos(dh[1][4]),
            0, 0, 0, 1;
    f(3) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) + std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), std::sin(dh[2][4])*std::cos(dh[2][3]), dh[2][2]*std::cos(dh[2][4]) + dh[2][0]*std::sin(dh[2][4])*std::cos(dh[2][3]),
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) - std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), std::cos(dh[2][3])*std::cos(dh[2][4]), -dh[2][2]*std::sin(dh[2][4]) + dh[2][0]*std::cos(dh[2][3])*std::cos(dh[2][4]),
            0, 0, 0, 1;
    f(4) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]) + std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]), std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]) - std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]), std::sin(dh[3][4])*std::cos(dh[3][3]), dh[3][2]*std::cos(dh[3][4]) + dh[3][0]*std::sin(dh[3][4])*std::cos(dh[3][3]),
            std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]), std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]), std::cos(dh[3][3])*std::cos(dh[3][4]), -dh[3][2]*std::sin(dh[3][4]) + dh[3][0]*std::cos(dh[3][3])*std::cos(dh[3][4]),
            0, 0, 0, 1;
    f(5) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    
}


inline void _fill_jac(JACS& j, JOINTS& q){
    j(0, 1) << -std::sin(q[0]), -std::cos(q[0]), 0, 0,
               std::cos(q[0]), -std::sin(q[0]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(1, 2) << -std::sin(q[1]), -std::cos(q[1]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[1]), -std::sin(q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 3) << -std::sin(q[2]), -std::cos(q[2]), 0, 0,
               std::cos(q[2]), -std::sin(q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(2, 4) << std::cos(q[2]), -std::sin(q[2]), 0, 0,
               std::cos(q[2] - 0.5*pi), std::cos(q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh){
    j(0, 1) << -std::sin(dh[0][1] + q[0]), -std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 2) << -std::sin(dh[1][1] + q[1]), -std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 3) << -std::sin(dh[2][1] + q[2]), -std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(dh[3][1] + q[2]), -std::cos(dh[3][1] + q[2]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    
}


inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh){
    j(0, 1) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), -std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) - std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) + std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 2) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), -std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) - std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) + std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 3) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), -std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) - std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) + std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]) - std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]), -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]) - std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]) + std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    
}


void combine_frames(FRAMES& f){
    f(1) = f(0) * f(1);
    f(2) = f(1) * f(2);
    f(3) = f(2) * f(3);
    f(4) = f(3) * f(4);
    f(5) = f(4) * f(5);
    
}


void combine_jacs(JACS& j, DICT& d){
    // 0
    j(0, 1) = d(0, 0) * j(0, 1);
    j(0, 2) = j(0, 1) * d(2, 2);
    j(0, 3) = j(0, 1) * d(2, 3);
    j(0, 4) = j(0, 1) * d(2, 4);
    j(0, 5) = j(0, 1) * d(2, 5);
    // 1
    j(1, 2) = d(0, 1) * j(1, 2);
    j(1, 3) = j(1, 2) * d(3, 3);
    j(1, 4) = j(1, 2) * d(3, 4);
    j(1, 5) = j(1, 2) * d(3, 5);
    // 2
    j(2, 3) = d(0, 2) * j(2, 3);
    j(2, 4) = j(2, 3) * d(4, 4) + d(0, 3) * j(2, 4);
    j(2, 5) = j(2, 4) * d(5, 5);
    
}


void combine_dict(FRAMES& f, DICT& d){
    // 5
    d(5, 5) = f(5);
    // 4
    d(4, 4) = f(4);
    d(4, 5) = f(4) * d(5, 5);
    // 3
    d(3, 3) = f(3);
    d(3, 4) = f(3) * d(4, 4);
    d(3, 5) = f(3) * d(4, 5);
    // 2
    d(2, 2) = f(2);
    d(2, 3) = f(2) * d(3, 3);
    d(2, 4) = f(2) * d(3, 4);
    d(2, 5) = f(2) * d(3, 5);
    // 1
    d(1, 1) = f(1);
    d(1, 2) = f(1) * d(2, 2);
    d(1, 3) = f(1) * d(2, 3);
    d(1, 4) = f(1) * d(2, 4);
    d(1, 5) = f(1) * d(2, 5);
    // 0
    d(0, 0) = f(0);
    d(0, 1) = f(0) * d(1, 1);
    d(0, 2) = f(0) * d(1, 2);
    d(0, 3) = f(0) * d(1, 3);
    d(0, 4) = f(0) * d(1, 4);
    d(0, 5) = f(0) * d(1, 5);
    
}

    