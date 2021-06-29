#include "Justin19.h"



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
            0, 0, 1., 0.58840,
            0, 0, 0, 1.;
    f(1) << std::cos(q[0]), -std::sin(q[0]), 0, 0,
            std::sin(q[0]), std::cos(q[0]), 0, 0,
            0, 0, 1, 0.10550,
            0, 0, 0, 1;
    f(2) << std::cos(q[1] - 0.5*pi), std::cos(q[1]), 0, 0,
            0, 0, 1, 0,
            std::cos(q[1]), -std::sin(q[1]), 0, 0,
            0, 0, 0, 1;
    f(3) << std::cos(q[2]), -std::sin(q[2]), 0, 0.3,
            std::sin(q[2]), std::cos(q[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(4) << std::cos(q[1] + q[2]), std::sin(q[1] + q[2]), 0, 0.3,
            -std::sin(q[1] + q[2]), std::cos(q[1] + q[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(5) << -0.866, 0, 0.5, 0.190,
            0, 1., 0, 0.0880,
            -0.5, 0, -0.866, -0.256,
            0, 0, 0, 1.;
    f(6) << std::cos(q[3]), -std::sin(q[3]), 0, 0,
            std::sin(q[3]), std::cos(q[3]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(7) << std::cos(q[4]), -std::sin(q[4]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[4]), std::cos(q[4]), 0, 0,
            0, 0, 0, 1;
    f(8) << std::cos(q[5] - 0.5*pi), std::cos(q[5]), 0, 0,
            0, 0, 1, 0.4,
            std::cos(q[5]), -std::sin(q[5]), 0, 0,
            0, 0, 0, 1;
    f(9) << std::cos(q[6]), -std::sin(q[6]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[6]), std::cos(q[6]), 0, 0,
            0, 0, 0, 1;
    f(10) << -std::cos(q[7]), std::sin(q[7]), 0, 0,
             0, 0, 1, 0.390,
             std::sin(q[7]), std::cos(q[7]), 0, 0,
             0, 0, 0, 1;
    f(11) << std::cos(q[8] + 0.5*pi), -std::cos(q[8]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[8] + 0.5*pi), std::cos(q[8] + 0.5*pi), 0, 0,
             0, 0, 0, 1;
    f(12) << std::cos(q[9] - 0.5*pi), std::cos(q[9]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[9] - 0.5*pi), std::cos(q[9] - 0.5*pi), 0, 0,
             0, 0, 0, 1;
    f(13) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(14) << 0.866, 0, 0.5, 0.190,
             0, 1., 0, 0.0880,
             -0.5, 0, 0.866, 0.256,
             0, 0, 0, 1.;
    f(15) << std::cos(q[10]), -std::sin(q[10]), 0, 0,
             -std::sin(q[10]), -std::cos(q[10]), 0, 0,
             0, 0, -1, 0,
             0, 0, 0, 1;
    f(16) << std::cos(q[11]), -std::sin(q[11]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[11]), std::cos(q[11]), 0, 0,
             0, 0, 0, 1;
    f(17) << std::cos(q[12] - 0.5*pi), std::cos(q[12]), 0, 0,
             0, 0, 1, -0.4,
             std::cos(q[12]), -std::sin(q[12]), 0, 0,
             0, 0, 0, 1;
    f(18) << std::cos(q[13]), -std::sin(q[13]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[13]), std::cos(q[13]), 0, 0,
             0, 0, 0, 1;
    f(19) << std::cos(q[14]), -std::sin(q[14]), 0, 0,
             0, 0, 1, -0.390,
             -std::sin(q[14]), -std::cos(q[14]), 0, 0,
             0, 0, 0, 1;
    f(20) << std::cos(q[15] + 0.5*pi), -std::cos(q[15]), 0, 0,
             0, 0, 1, 0,
             -std::cos(q[15]), std::sin(q[15]), 0, 0,
             0, 0, 0, 1;
    f(21) << std::cos(q[16] - 0.5*pi), std::cos(q[16]), 0, 0,
             0, 0, 1, 0,
             std::cos(q[16]), -std::sin(q[16]), 0, 0,
             0, 0, 0, 1;
    f(22) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(23) << 0, 0, 1., 0.235,
             1., 0, 0, 0.0880,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(24) << std::cos(q[17]), -std::sin(q[17]), 0, 0,
             std::sin(q[17]), std::cos(q[17]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(25) << std::cos(q[18]), -std::sin(q[18]), 0, 0,
             0, 0, 1, 0,
             -std::sin(q[18]), -std::cos(q[18]), 0, 0,
             0, 0, 0, 1;
    f(26) << 1., 0, 0, 0,
             0, 0, -1., 0,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0.58840,
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
    f(4) << std::cos(-dh[3][1] + q[1] + q[2]), std::sin(-dh[3][1] + q[1] + q[2]), 0, dh[3][2],
            -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2]), std::sin(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), std::cos(dh[3][3]), dh[3][0]*std::cos(dh[3][3]),
            0, 0, 0, 1;
    f(5) << -0.866, 0, 0.5, 0.190,
            0, 1., 0, 0.0880,
            -0.5, 0, -0.866, -0.256,
            0, 0, 0, 1.;
    f(6) << std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3]), 0, dh[4][2],
            std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[3]), std::sin(dh[4][3])*std::cos(dh[4][1] + q[3]), std::cos(dh[4][3]), dh[4][0]*std::cos(dh[4][3]),
            0, 0, 0, 1;
    f(7) << std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4]), 0, dh[5][2],
            std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[4]), std::sin(dh[5][3])*std::cos(dh[5][1] + q[4]), std::cos(dh[5][3]), dh[5][0]*std::cos(dh[5][3]),
            0, 0, 0, 1;
    f(8) << std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5]), 0, dh[6][2],
            std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
            std::sin(dh[6][3])*std::sin(dh[6][1] + q[5]), std::sin(dh[6][3])*std::cos(dh[6][1] + q[5]), std::cos(dh[6][3]), dh[6][0]*std::cos(dh[6][3]),
            0, 0, 0, 1;
    f(9) << std::cos(dh[7][1] + q[6]), -std::sin(dh[7][1] + q[6]), 0, dh[7][2],
            std::sin(dh[7][1] + q[6])*std::cos(dh[7][3]), std::cos(dh[7][3])*std::cos(dh[7][1] + q[6]), -std::sin(dh[7][3]), -dh[7][0]*std::sin(dh[7][3]),
            std::sin(dh[7][3])*std::sin(dh[7][1] + q[6]), std::sin(dh[7][3])*std::cos(dh[7][1] + q[6]), std::cos(dh[7][3]), dh[7][0]*std::cos(dh[7][3]),
            0, 0, 0, 1;
    f(10) << std::cos(dh[8][1] + q[7]), -std::sin(dh[8][1] + q[7]), 0, dh[8][2],
             std::sin(dh[8][1] + q[7])*std::cos(dh[8][3]), std::cos(dh[8][3])*std::cos(dh[8][1] + q[7]), -std::sin(dh[8][3]), -dh[8][0]*std::sin(dh[8][3]),
             std::sin(dh[8][3])*std::sin(dh[8][1] + q[7]), std::sin(dh[8][3])*std::cos(dh[8][1] + q[7]), std::cos(dh[8][3]), dh[8][0]*std::cos(dh[8][3]),
             0, 0, 0, 1;
    f(11) << std::cos(dh[9][1] + q[8]), -std::sin(dh[9][1] + q[8]), 0, dh[9][2],
             std::sin(dh[9][1] + q[8])*std::cos(dh[9][3]), std::cos(dh[9][3])*std::cos(dh[9][1] + q[8]), -std::sin(dh[9][3]), -dh[9][0]*std::sin(dh[9][3]),
             std::sin(dh[9][3])*std::sin(dh[9][1] + q[8]), std::sin(dh[9][3])*std::cos(dh[9][1] + q[8]), std::cos(dh[9][3]), dh[9][0]*std::cos(dh[9][3]),
             0, 0, 0, 1;
    f(12) << std::cos(dh[10][1] + q[9]), -std::sin(dh[10][1] + q[9]), 0, dh[10][2],
             std::sin(dh[10][1] + q[9])*std::cos(dh[10][3]), std::cos(dh[10][3])*std::cos(dh[10][1] + q[9]), -std::sin(dh[10][3]), -dh[10][0]*std::sin(dh[10][3]),
             std::sin(dh[10][3])*std::sin(dh[10][1] + q[9]), std::sin(dh[10][3])*std::cos(dh[10][1] + q[9]), std::cos(dh[10][3]), dh[10][0]*std::cos(dh[10][3]),
             0, 0, 0, 1;
    f(13) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(14) << 0.866, 0, 0.5, 0.190,
             0, 1., 0, 0.0880,
             -0.5, 0, 0.866, 0.256,
             0, 0, 0, 1.;
    f(15) << std::cos(dh[11][1] + q[10]), -std::sin(dh[11][1] + q[10]), 0, dh[11][2],
             std::sin(dh[11][1] + q[10])*std::cos(dh[11][3]), std::cos(dh[11][3])*std::cos(dh[11][1] + q[10]), -std::sin(dh[11][3]), -dh[11][0]*std::sin(dh[11][3]),
             std::sin(dh[11][3])*std::sin(dh[11][1] + q[10]), std::sin(dh[11][3])*std::cos(dh[11][1] + q[10]), std::cos(dh[11][3]), dh[11][0]*std::cos(dh[11][3]),
             0, 0, 0, 1;
    f(16) << std::cos(dh[12][1] + q[11]), -std::sin(dh[12][1] + q[11]), 0, dh[12][2],
             std::sin(dh[12][1] + q[11])*std::cos(dh[12][3]), std::cos(dh[12][3])*std::cos(dh[12][1] + q[11]), -std::sin(dh[12][3]), -dh[12][0]*std::sin(dh[12][3]),
             std::sin(dh[12][3])*std::sin(dh[12][1] + q[11]), std::sin(dh[12][3])*std::cos(dh[12][1] + q[11]), std::cos(dh[12][3]), dh[12][0]*std::cos(dh[12][3]),
             0, 0, 0, 1;
    f(17) << std::cos(dh[13][1] + q[12]), -std::sin(dh[13][1] + q[12]), 0, dh[13][2],
             std::sin(dh[13][1] + q[12])*std::cos(dh[13][3]), std::cos(dh[13][3])*std::cos(dh[13][1] + q[12]), -std::sin(dh[13][3]), -dh[13][0]*std::sin(dh[13][3]),
             std::sin(dh[13][3])*std::sin(dh[13][1] + q[12]), std::sin(dh[13][3])*std::cos(dh[13][1] + q[12]), std::cos(dh[13][3]), dh[13][0]*std::cos(dh[13][3]),
             0, 0, 0, 1;
    f(18) << std::cos(dh[14][1] + q[13]), -std::sin(dh[14][1] + q[13]), 0, dh[14][2],
             std::sin(dh[14][1] + q[13])*std::cos(dh[14][3]), std::cos(dh[14][3])*std::cos(dh[14][1] + q[13]), -std::sin(dh[14][3]), -dh[14][0]*std::sin(dh[14][3]),
             std::sin(dh[14][3])*std::sin(dh[14][1] + q[13]), std::sin(dh[14][3])*std::cos(dh[14][1] + q[13]), std::cos(dh[14][3]), dh[14][0]*std::cos(dh[14][3]),
             0, 0, 0, 1;
    f(19) << std::cos(dh[15][1] + q[14]), -std::sin(dh[15][1] + q[14]), 0, dh[15][2],
             std::sin(dh[15][1] + q[14])*std::cos(dh[15][3]), std::cos(dh[15][3])*std::cos(dh[15][1] + q[14]), -std::sin(dh[15][3]), -dh[15][0]*std::sin(dh[15][3]),
             std::sin(dh[15][3])*std::sin(dh[15][1] + q[14]), std::sin(dh[15][3])*std::cos(dh[15][1] + q[14]), std::cos(dh[15][3]), dh[15][0]*std::cos(dh[15][3]),
             0, 0, 0, 1;
    f(20) << std::cos(dh[16][1] + q[15]), -std::sin(dh[16][1] + q[15]), 0, dh[16][2],
             std::sin(dh[16][1] + q[15])*std::cos(dh[16][3]), std::cos(dh[16][3])*std::cos(dh[16][1] + q[15]), -std::sin(dh[16][3]), -dh[16][0]*std::sin(dh[16][3]),
             std::sin(dh[16][3])*std::sin(dh[16][1] + q[15]), std::sin(dh[16][3])*std::cos(dh[16][1] + q[15]), std::cos(dh[16][3]), dh[16][0]*std::cos(dh[16][3]),
             0, 0, 0, 1;
    f(21) << std::cos(dh[17][1] + q[16]), -std::sin(dh[17][1] + q[16]), 0, dh[17][2],
             std::sin(dh[17][1] + q[16])*std::cos(dh[17][3]), std::cos(dh[17][3])*std::cos(dh[17][1] + q[16]), -std::sin(dh[17][3]), -dh[17][0]*std::sin(dh[17][3]),
             std::sin(dh[17][3])*std::sin(dh[17][1] + q[16]), std::sin(dh[17][3])*std::cos(dh[17][1] + q[16]), std::cos(dh[17][3]), dh[17][0]*std::cos(dh[17][3]),
             0, 0, 0, 1;
    f(22) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(23) << 0, 0, 1., 0.235,
             1., 0, 0, 0.0880,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(24) << std::cos(dh[18][1] + q[17]), -std::sin(dh[18][1] + q[17]), 0, dh[18][2],
             std::sin(dh[18][1] + q[17])*std::cos(dh[18][3]), std::cos(dh[18][3])*std::cos(dh[18][1] + q[17]), -std::sin(dh[18][3]), -dh[18][0]*std::sin(dh[18][3]),
             std::sin(dh[18][3])*std::sin(dh[18][1] + q[17]), std::sin(dh[18][3])*std::cos(dh[18][1] + q[17]), std::cos(dh[18][3]), dh[18][0]*std::cos(dh[18][3]),
             0, 0, 0, 1;
    f(25) << std::cos(dh[19][1] + q[18]), -std::sin(dh[19][1] + q[18]), 0, dh[19][2],
             std::sin(dh[19][1] + q[18])*std::cos(dh[19][3]), std::cos(dh[19][3])*std::cos(dh[19][1] + q[18]), -std::sin(dh[19][3]), -dh[19][0]*std::sin(dh[19][3]),
             std::sin(dh[19][3])*std::sin(dh[19][1] + q[18]), std::sin(dh[19][3])*std::cos(dh[19][1] + q[18]), std::cos(dh[19][3]), dh[19][0]*std::cos(dh[19][3]),
             0, 0, 0, 1;
    f(26) << 1., 0, 0, 0,
             0, 0, -1., 0,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0.58840,
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
    f(4) << -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]) + std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) + std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]), std::sin(dh[3][4])*std::cos(dh[3][3]), dh[3][2]*std::cos(dh[3][4]) + dh[3][0]*std::sin(dh[3][4])*std::cos(dh[3][3]),
            -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) - std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]), std::cos(dh[3][3])*std::cos(dh[3][4]), -dh[3][2]*std::sin(dh[3][4]) + dh[3][0]*std::cos(dh[3][3])*std::cos(dh[3][4]),
            0, 0, 0, 1;
    f(5) << -0.866, 0, 0.5, 0.190,
            0, 1., 0, 0.0880,
            -0.5, 0, -0.866, -0.256,
            0, 0, 0, 1.;
    f(6) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]) + std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]), std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]) - std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]), std::sin(dh[4][4])*std::cos(dh[4][3]), dh[4][2]*std::cos(dh[4][4]) + dh[4][0]*std::sin(dh[4][4])*std::cos(dh[4][3]),
            std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]) - std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]), std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]), std::cos(dh[4][3])*std::cos(dh[4][4]), -dh[4][2]*std::sin(dh[4][4]) + dh[4][0]*std::cos(dh[4][3])*std::cos(dh[4][4]),
            0, 0, 0, 1;
    f(7) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]) + std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]), std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]) - std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]), std::sin(dh[5][4])*std::cos(dh[5][3]), dh[5][2]*std::cos(dh[5][4]) + dh[5][0]*std::sin(dh[5][4])*std::cos(dh[5][3]),
            std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]) - std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]), std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]), std::cos(dh[5][3])*std::cos(dh[5][4]), -dh[5][2]*std::sin(dh[5][4]) + dh[5][0]*std::cos(dh[5][3])*std::cos(dh[5][4]),
            0, 0, 0, 1;
    f(8) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]) + std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]), std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]) - std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]), std::sin(dh[6][4])*std::cos(dh[6][3]), dh[6][2]*std::cos(dh[6][4]) + dh[6][0]*std::sin(dh[6][4])*std::cos(dh[6][3]),
            std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
            std::sin(dh[6][3])*std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]) - std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]), std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]), std::cos(dh[6][3])*std::cos(dh[6][4]), -dh[6][2]*std::sin(dh[6][4]) + dh[6][0]*std::cos(dh[6][3])*std::cos(dh[6][4]),
            0, 0, 0, 1;
    f(9) << std::sin(dh[7][3])*std::sin(dh[7][4])*std::sin(dh[7][1] + q[6]) + std::cos(dh[7][4])*std::cos(dh[7][1] + q[6]), std::sin(dh[7][3])*std::sin(dh[7][4])*std::cos(dh[7][1] + q[6]) - std::sin(dh[7][1] + q[6])*std::cos(dh[7][4]), std::sin(dh[7][4])*std::cos(dh[7][3]), dh[7][2]*std::cos(dh[7][4]) + dh[7][0]*std::sin(dh[7][4])*std::cos(dh[7][3]),
            std::sin(dh[7][1] + q[6])*std::cos(dh[7][3]), std::cos(dh[7][3])*std::cos(dh[7][1] + q[6]), -std::sin(dh[7][3]), -dh[7][0]*std::sin(dh[7][3]),
            std::sin(dh[7][3])*std::sin(dh[7][1] + q[6])*std::cos(dh[7][4]) - std::sin(dh[7][4])*std::cos(dh[7][1] + q[6]), std::sin(dh[7][3])*std::cos(dh[7][4])*std::cos(dh[7][1] + q[6]) + std::sin(dh[7][4])*std::sin(dh[7][1] + q[6]), std::cos(dh[7][3])*std::cos(dh[7][4]), -dh[7][2]*std::sin(dh[7][4]) + dh[7][0]*std::cos(dh[7][3])*std::cos(dh[7][4]),
            0, 0, 0, 1;
    f(10) << std::sin(dh[8][3])*std::sin(dh[8][4])*std::sin(dh[8][1] + q[7]) + std::cos(dh[8][4])*std::cos(dh[8][1] + q[7]), std::sin(dh[8][3])*std::sin(dh[8][4])*std::cos(dh[8][1] + q[7]) - std::sin(dh[8][1] + q[7])*std::cos(dh[8][4]), std::sin(dh[8][4])*std::cos(dh[8][3]), dh[8][2]*std::cos(dh[8][4]) + dh[8][0]*std::sin(dh[8][4])*std::cos(dh[8][3]),
             std::sin(dh[8][1] + q[7])*std::cos(dh[8][3]), std::cos(dh[8][3])*std::cos(dh[8][1] + q[7]), -std::sin(dh[8][3]), -dh[8][0]*std::sin(dh[8][3]),
             std::sin(dh[8][3])*std::sin(dh[8][1] + q[7])*std::cos(dh[8][4]) - std::sin(dh[8][4])*std::cos(dh[8][1] + q[7]), std::sin(dh[8][3])*std::cos(dh[8][4])*std::cos(dh[8][1] + q[7]) + std::sin(dh[8][4])*std::sin(dh[8][1] + q[7]), std::cos(dh[8][3])*std::cos(dh[8][4]), -dh[8][2]*std::sin(dh[8][4]) + dh[8][0]*std::cos(dh[8][3])*std::cos(dh[8][4]),
             0, 0, 0, 1;
    f(11) << std::sin(dh[9][3])*std::sin(dh[9][4])*std::sin(dh[9][1] + q[8]) + std::cos(dh[9][4])*std::cos(dh[9][1] + q[8]), std::sin(dh[9][3])*std::sin(dh[9][4])*std::cos(dh[9][1] + q[8]) - std::sin(dh[9][1] + q[8])*std::cos(dh[9][4]), std::sin(dh[9][4])*std::cos(dh[9][3]), dh[9][2]*std::cos(dh[9][4]) + dh[9][0]*std::sin(dh[9][4])*std::cos(dh[9][3]),
             std::sin(dh[9][1] + q[8])*std::cos(dh[9][3]), std::cos(dh[9][3])*std::cos(dh[9][1] + q[8]), -std::sin(dh[9][3]), -dh[9][0]*std::sin(dh[9][3]),
             std::sin(dh[9][3])*std::sin(dh[9][1] + q[8])*std::cos(dh[9][4]) - std::sin(dh[9][4])*std::cos(dh[9][1] + q[8]), std::sin(dh[9][3])*std::cos(dh[9][4])*std::cos(dh[9][1] + q[8]) + std::sin(dh[9][4])*std::sin(dh[9][1] + q[8]), std::cos(dh[9][3])*std::cos(dh[9][4]), -dh[9][2]*std::sin(dh[9][4]) + dh[9][0]*std::cos(dh[9][3])*std::cos(dh[9][4]),
             0, 0, 0, 1;
    f(12) << std::sin(dh[10][3])*std::sin(dh[10][4])*std::sin(dh[10][1] + q[9]) + std::cos(dh[10][4])*std::cos(dh[10][1] + q[9]), std::sin(dh[10][3])*std::sin(dh[10][4])*std::cos(dh[10][1] + q[9]) - std::sin(dh[10][1] + q[9])*std::cos(dh[10][4]), std::sin(dh[10][4])*std::cos(dh[10][3]), dh[10][2]*std::cos(dh[10][4]) + dh[10][0]*std::sin(dh[10][4])*std::cos(dh[10][3]),
             std::sin(dh[10][1] + q[9])*std::cos(dh[10][3]), std::cos(dh[10][3])*std::cos(dh[10][1] + q[9]), -std::sin(dh[10][3]), -dh[10][0]*std::sin(dh[10][3]),
             std::sin(dh[10][3])*std::sin(dh[10][1] + q[9])*std::cos(dh[10][4]) - std::sin(dh[10][4])*std::cos(dh[10][1] + q[9]), std::sin(dh[10][3])*std::cos(dh[10][4])*std::cos(dh[10][1] + q[9]) + std::sin(dh[10][4])*std::sin(dh[10][1] + q[9]), std::cos(dh[10][3])*std::cos(dh[10][4]), -dh[10][2]*std::sin(dh[10][4]) + dh[10][0]*std::cos(dh[10][3])*std::cos(dh[10][4]),
             0, 0, 0, 1;
    f(13) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(14) << 0.866, 0, 0.5, 0.190,
             0, 1., 0, 0.0880,
             -0.5, 0, 0.866, 0.256,
             0, 0, 0, 1.;
    f(15) << std::sin(dh[11][3])*std::sin(dh[11][4])*std::sin(dh[11][1] + q[10]) + std::cos(dh[11][4])*std::cos(dh[11][1] + q[10]), std::sin(dh[11][3])*std::sin(dh[11][4])*std::cos(dh[11][1] + q[10]) - std::sin(dh[11][1] + q[10])*std::cos(dh[11][4]), std::sin(dh[11][4])*std::cos(dh[11][3]), dh[11][2]*std::cos(dh[11][4]) + dh[11][0]*std::sin(dh[11][4])*std::cos(dh[11][3]),
             std::sin(dh[11][1] + q[10])*std::cos(dh[11][3]), std::cos(dh[11][3])*std::cos(dh[11][1] + q[10]), -std::sin(dh[11][3]), -dh[11][0]*std::sin(dh[11][3]),
             std::sin(dh[11][3])*std::sin(dh[11][1] + q[10])*std::cos(dh[11][4]) - std::sin(dh[11][4])*std::cos(dh[11][1] + q[10]), std::sin(dh[11][3])*std::cos(dh[11][4])*std::cos(dh[11][1] + q[10]) + std::sin(dh[11][4])*std::sin(dh[11][1] + q[10]), std::cos(dh[11][3])*std::cos(dh[11][4]), -dh[11][2]*std::sin(dh[11][4]) + dh[11][0]*std::cos(dh[11][3])*std::cos(dh[11][4]),
             0, 0, 0, 1;
    f(16) << std::sin(dh[12][3])*std::sin(dh[12][4])*std::sin(dh[12][1] + q[11]) + std::cos(dh[12][4])*std::cos(dh[12][1] + q[11]), std::sin(dh[12][3])*std::sin(dh[12][4])*std::cos(dh[12][1] + q[11]) - std::sin(dh[12][1] + q[11])*std::cos(dh[12][4]), std::sin(dh[12][4])*std::cos(dh[12][3]), dh[12][2]*std::cos(dh[12][4]) + dh[12][0]*std::sin(dh[12][4])*std::cos(dh[12][3]),
             std::sin(dh[12][1] + q[11])*std::cos(dh[12][3]), std::cos(dh[12][3])*std::cos(dh[12][1] + q[11]), -std::sin(dh[12][3]), -dh[12][0]*std::sin(dh[12][3]),
             std::sin(dh[12][3])*std::sin(dh[12][1] + q[11])*std::cos(dh[12][4]) - std::sin(dh[12][4])*std::cos(dh[12][1] + q[11]), std::sin(dh[12][3])*std::cos(dh[12][4])*std::cos(dh[12][1] + q[11]) + std::sin(dh[12][4])*std::sin(dh[12][1] + q[11]), std::cos(dh[12][3])*std::cos(dh[12][4]), -dh[12][2]*std::sin(dh[12][4]) + dh[12][0]*std::cos(dh[12][3])*std::cos(dh[12][4]),
             0, 0, 0, 1;
    f(17) << std::sin(dh[13][3])*std::sin(dh[13][4])*std::sin(dh[13][1] + q[12]) + std::cos(dh[13][4])*std::cos(dh[13][1] + q[12]), std::sin(dh[13][3])*std::sin(dh[13][4])*std::cos(dh[13][1] + q[12]) - std::sin(dh[13][1] + q[12])*std::cos(dh[13][4]), std::sin(dh[13][4])*std::cos(dh[13][3]), dh[13][2]*std::cos(dh[13][4]) + dh[13][0]*std::sin(dh[13][4])*std::cos(dh[13][3]),
             std::sin(dh[13][1] + q[12])*std::cos(dh[13][3]), std::cos(dh[13][3])*std::cos(dh[13][1] + q[12]), -std::sin(dh[13][3]), -dh[13][0]*std::sin(dh[13][3]),
             std::sin(dh[13][3])*std::sin(dh[13][1] + q[12])*std::cos(dh[13][4]) - std::sin(dh[13][4])*std::cos(dh[13][1] + q[12]), std::sin(dh[13][3])*std::cos(dh[13][4])*std::cos(dh[13][1] + q[12]) + std::sin(dh[13][4])*std::sin(dh[13][1] + q[12]), std::cos(dh[13][3])*std::cos(dh[13][4]), -dh[13][2]*std::sin(dh[13][4]) + dh[13][0]*std::cos(dh[13][3])*std::cos(dh[13][4]),
             0, 0, 0, 1;
    f(18) << std::sin(dh[14][3])*std::sin(dh[14][4])*std::sin(dh[14][1] + q[13]) + std::cos(dh[14][4])*std::cos(dh[14][1] + q[13]), std::sin(dh[14][3])*std::sin(dh[14][4])*std::cos(dh[14][1] + q[13]) - std::sin(dh[14][1] + q[13])*std::cos(dh[14][4]), std::sin(dh[14][4])*std::cos(dh[14][3]), dh[14][2]*std::cos(dh[14][4]) + dh[14][0]*std::sin(dh[14][4])*std::cos(dh[14][3]),
             std::sin(dh[14][1] + q[13])*std::cos(dh[14][3]), std::cos(dh[14][3])*std::cos(dh[14][1] + q[13]), -std::sin(dh[14][3]), -dh[14][0]*std::sin(dh[14][3]),
             std::sin(dh[14][3])*std::sin(dh[14][1] + q[13])*std::cos(dh[14][4]) - std::sin(dh[14][4])*std::cos(dh[14][1] + q[13]), std::sin(dh[14][3])*std::cos(dh[14][4])*std::cos(dh[14][1] + q[13]) + std::sin(dh[14][4])*std::sin(dh[14][1] + q[13]), std::cos(dh[14][3])*std::cos(dh[14][4]), -dh[14][2]*std::sin(dh[14][4]) + dh[14][0]*std::cos(dh[14][3])*std::cos(dh[14][4]),
             0, 0, 0, 1;
    f(19) << std::sin(dh[15][3])*std::sin(dh[15][4])*std::sin(dh[15][1] + q[14]) + std::cos(dh[15][4])*std::cos(dh[15][1] + q[14]), std::sin(dh[15][3])*std::sin(dh[15][4])*std::cos(dh[15][1] + q[14]) - std::sin(dh[15][1] + q[14])*std::cos(dh[15][4]), std::sin(dh[15][4])*std::cos(dh[15][3]), dh[15][2]*std::cos(dh[15][4]) + dh[15][0]*std::sin(dh[15][4])*std::cos(dh[15][3]),
             std::sin(dh[15][1] + q[14])*std::cos(dh[15][3]), std::cos(dh[15][3])*std::cos(dh[15][1] + q[14]), -std::sin(dh[15][3]), -dh[15][0]*std::sin(dh[15][3]),
             std::sin(dh[15][3])*std::sin(dh[15][1] + q[14])*std::cos(dh[15][4]) - std::sin(dh[15][4])*std::cos(dh[15][1] + q[14]), std::sin(dh[15][3])*std::cos(dh[15][4])*std::cos(dh[15][1] + q[14]) + std::sin(dh[15][4])*std::sin(dh[15][1] + q[14]), std::cos(dh[15][3])*std::cos(dh[15][4]), -dh[15][2]*std::sin(dh[15][4]) + dh[15][0]*std::cos(dh[15][3])*std::cos(dh[15][4]),
             0, 0, 0, 1;
    f(20) << std::sin(dh[16][3])*std::sin(dh[16][4])*std::sin(dh[16][1] + q[15]) + std::cos(dh[16][4])*std::cos(dh[16][1] + q[15]), std::sin(dh[16][3])*std::sin(dh[16][4])*std::cos(dh[16][1] + q[15]) - std::sin(dh[16][1] + q[15])*std::cos(dh[16][4]), std::sin(dh[16][4])*std::cos(dh[16][3]), dh[16][2]*std::cos(dh[16][4]) + dh[16][0]*std::sin(dh[16][4])*std::cos(dh[16][3]),
             std::sin(dh[16][1] + q[15])*std::cos(dh[16][3]), std::cos(dh[16][3])*std::cos(dh[16][1] + q[15]), -std::sin(dh[16][3]), -dh[16][0]*std::sin(dh[16][3]),
             std::sin(dh[16][3])*std::sin(dh[16][1] + q[15])*std::cos(dh[16][4]) - std::sin(dh[16][4])*std::cos(dh[16][1] + q[15]), std::sin(dh[16][3])*std::cos(dh[16][4])*std::cos(dh[16][1] + q[15]) + std::sin(dh[16][4])*std::sin(dh[16][1] + q[15]), std::cos(dh[16][3])*std::cos(dh[16][4]), -dh[16][2]*std::sin(dh[16][4]) + dh[16][0]*std::cos(dh[16][3])*std::cos(dh[16][4]),
             0, 0, 0, 1;
    f(21) << std::sin(dh[17][3])*std::sin(dh[17][4])*std::sin(dh[17][1] + q[16]) + std::cos(dh[17][4])*std::cos(dh[17][1] + q[16]), std::sin(dh[17][3])*std::sin(dh[17][4])*std::cos(dh[17][1] + q[16]) - std::sin(dh[17][1] + q[16])*std::cos(dh[17][4]), std::sin(dh[17][4])*std::cos(dh[17][3]), dh[17][2]*std::cos(dh[17][4]) + dh[17][0]*std::sin(dh[17][4])*std::cos(dh[17][3]),
             std::sin(dh[17][1] + q[16])*std::cos(dh[17][3]), std::cos(dh[17][3])*std::cos(dh[17][1] + q[16]), -std::sin(dh[17][3]), -dh[17][0]*std::sin(dh[17][3]),
             std::sin(dh[17][3])*std::sin(dh[17][1] + q[16])*std::cos(dh[17][4]) - std::sin(dh[17][4])*std::cos(dh[17][1] + q[16]), std::sin(dh[17][3])*std::cos(dh[17][4])*std::cos(dh[17][1] + q[16]) + std::sin(dh[17][4])*std::sin(dh[17][1] + q[16]), std::cos(dh[17][3])*std::cos(dh[17][4]), -dh[17][2]*std::sin(dh[17][4]) + dh[17][0]*std::cos(dh[17][3])*std::cos(dh[17][4]),
             0, 0, 0, 1;
    f(22) << -1., 0, 0, 0,
             0, 0, 1., 0.118,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(23) << 0, 0, 1., 0.235,
             1., 0, 0, 0.0880,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(24) << std::sin(dh[18][3])*std::sin(dh[18][4])*std::sin(dh[18][1] + q[17]) + std::cos(dh[18][4])*std::cos(dh[18][1] + q[17]), std::sin(dh[18][3])*std::sin(dh[18][4])*std::cos(dh[18][1] + q[17]) - std::sin(dh[18][1] + q[17])*std::cos(dh[18][4]), std::sin(dh[18][4])*std::cos(dh[18][3]), dh[18][2]*std::cos(dh[18][4]) + dh[18][0]*std::sin(dh[18][4])*std::cos(dh[18][3]),
             std::sin(dh[18][1] + q[17])*std::cos(dh[18][3]), std::cos(dh[18][3])*std::cos(dh[18][1] + q[17]), -std::sin(dh[18][3]), -dh[18][0]*std::sin(dh[18][3]),
             std::sin(dh[18][3])*std::sin(dh[18][1] + q[17])*std::cos(dh[18][4]) - std::sin(dh[18][4])*std::cos(dh[18][1] + q[17]), std::sin(dh[18][3])*std::cos(dh[18][4])*std::cos(dh[18][1] + q[17]) + std::sin(dh[18][4])*std::sin(dh[18][1] + q[17]), std::cos(dh[18][3])*std::cos(dh[18][4]), -dh[18][2]*std::sin(dh[18][4]) + dh[18][0]*std::cos(dh[18][3])*std::cos(dh[18][4]),
             0, 0, 0, 1;
    f(25) << std::sin(dh[19][3])*std::sin(dh[19][4])*std::sin(dh[19][1] + q[18]) + std::cos(dh[19][4])*std::cos(dh[19][1] + q[18]), std::sin(dh[19][3])*std::sin(dh[19][4])*std::cos(dh[19][1] + q[18]) - std::sin(dh[19][1] + q[18])*std::cos(dh[19][4]), std::sin(dh[19][4])*std::cos(dh[19][3]), dh[19][2]*std::cos(dh[19][4]) + dh[19][0]*std::sin(dh[19][4])*std::cos(dh[19][3]),
             std::sin(dh[19][1] + q[18])*std::cos(dh[19][3]), std::cos(dh[19][3])*std::cos(dh[19][1] + q[18]), -std::sin(dh[19][3]), -dh[19][0]*std::sin(dh[19][3]),
             std::sin(dh[19][3])*std::sin(dh[19][1] + q[18])*std::cos(dh[19][4]) - std::sin(dh[19][4])*std::cos(dh[19][1] + q[18]), std::sin(dh[19][3])*std::cos(dh[19][4])*std::cos(dh[19][1] + q[18]) + std::sin(dh[19][4])*std::sin(dh[19][1] + q[18]), std::cos(dh[19][3])*std::cos(dh[19][4]), -dh[19][2]*std::sin(dh[19][4]) + dh[19][0]*std::cos(dh[19][3])*std::cos(dh[19][4]),
             0, 0, 0, 1;
    f(26) << 1., 0, 0, 0,
             0, 0, -1., 0,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void _fill_jac(JACS& j, JOINTS& q){
    j(0, 1) << -std::sin(q[0]), -std::cos(q[0]), 0, 0,
               std::cos(q[0]), -std::sin(q[0]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(1, 2) << std::cos(q[1]), -std::sin(q[1]), 0, 0,
               0, 0, 0, 0,
               -std::sin(q[1]), -std::cos(q[1]), 0, 0,
               0, 0, 0, 0;
    j(1, 4) << -std::sin(q[1] + q[2]), std::cos(q[1] + q[2]), 0, 0,
               -std::cos(q[1] + q[2]), -std::sin(q[1] + q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(2, 3) << -std::sin(q[2]), -std::cos(q[2]), 0, 0,
               std::cos(q[2]), -std::sin(q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(q[1] + q[2]), std::cos(q[1] + q[2]), 0, 0,
               -std::cos(q[1] + q[2]), -std::sin(q[1] + q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(3, 6) << -std::sin(q[3]), -std::cos(q[3]), 0, 0,
               std::cos(q[3]), -std::sin(q[3]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(4, 7) << -std::sin(q[4]), -std::cos(q[4]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[4]), -std::sin(q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 8) << std::cos(q[5]), -std::sin(q[5]), 0, 0,
               0, 0, 0, 0,
               -std::sin(q[5]), -std::cos(q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 9) << -std::sin(q[6]), -std::cos(q[6]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[6]), -std::sin(q[6]), 0, 0,
               0, 0, 0, 0;
    j(7, 10) << std::sin(q[7]), std::cos(q[7]), 0, 0,
                0, 0, 0, 0,
                std::cos(q[7]), -std::sin(q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 11) << -std::cos(q[8]), std::sin(q[8]), 0, 0,
                0, 0, 0, 0,
                std::cos(q[8] + 0.5*pi), -std::cos(q[8]), 0, 0,
                0, 0, 0, 0;
    j(9, 12) << std::cos(q[9]), -std::sin(q[9]), 0, 0,
                0, 0, 0, 0,
                std::cos(q[9] - 0.5*pi), std::cos(q[9]), 0, 0,
                0, 0, 0, 0;
    j(10, 15) << -std::sin(q[10]), -std::cos(q[10]), 0, 0,
                 -std::cos(q[10]), std::sin(q[10]), 0, 0,
                 0, 0, 0, 0,
                 0, 0, 0, 0;
    j(11, 16) << -std::sin(q[11]), -std::cos(q[11]), 0, 0,
                 0, 0, 0, 0,
                 std::cos(q[11]), -std::sin(q[11]), 0, 0,
                 0, 0, 0, 0;
    j(12, 17) << std::cos(q[12]), -std::sin(q[12]), 0, 0,
                 0, 0, 0, 0,
                 -std::sin(q[12]), -std::cos(q[12]), 0, 0,
                 0, 0, 0, 0;
    j(13, 18) << -std::sin(q[13]), -std::cos(q[13]), 0, 0,
                 0, 0, 0, 0,
                 std::cos(q[13]), -std::sin(q[13]), 0, 0,
                 0, 0, 0, 0;
    j(14, 19) << -std::sin(q[14]), -std::cos(q[14]), 0, 0,
                 0, 0, 0, 0,
                 -std::cos(q[14]), std::sin(q[14]), 0, 0,
                 0, 0, 0, 0;
    j(15, 20) << -std::cos(q[15]), std::sin(q[15]), 0, 0,
                 0, 0, 0, 0,
                 std::sin(q[15]), std::cos(q[15]), 0, 0,
                 0, 0, 0, 0;
    j(16, 21) << std::cos(q[16]), -std::sin(q[16]), 0, 0,
                 0, 0, 0, 0,
                 -std::sin(q[16]), -std::cos(q[16]), 0, 0,
                 0, 0, 0, 0;
    j(17, 24) << -std::sin(q[17]), -std::cos(q[17]), 0, 0,
                 std::cos(q[17]), -std::sin(q[17]), 0, 0,
                 0, 0, 0, 0,
                 0, 0, 0, 0;
    j(18, 25) << -std::sin(q[18]), -std::cos(q[18]), 0, 0,
                 0, 0, 0, 0,
                 -std::cos(q[18]), std::sin(q[18]), 0, 0,
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
    j(1, 4) << -std::sin(-dh[3][1] + q[1] + q[2]), std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               -std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), 0, 0,
               -std::sin(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 3) << -std::sin(dh[2][1] + q[2]), -std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(-dh[3][1] + q[1] + q[2]), std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               -std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), 0, 0,
               -std::sin(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 6) << -std::sin(dh[4][1] + q[3]), -std::cos(dh[4][1] + q[3]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 7) << -std::sin(dh[5][1] + q[4]), -std::cos(dh[5][1] + q[4]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 8) << -std::sin(dh[6][1] + q[5]), -std::cos(dh[6][1] + q[5]), 0, 0,
               std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), 0, 0,
               std::sin(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 9) << -std::sin(dh[7][1] + q[6]), -std::cos(dh[7][1] + q[6]), 0, 0,
               std::cos(dh[7][3])*std::cos(dh[7][1] + q[6]), -std::sin(dh[7][1] + q[6])*std::cos(dh[7][3]), 0, 0,
               std::sin(dh[7][3])*std::cos(dh[7][1] + q[6]), -std::sin(dh[7][3])*std::sin(dh[7][1] + q[6]), 0, 0,
               0, 0, 0, 0;
    j(7, 10) << -std::sin(dh[8][1] + q[7]), -std::cos(dh[8][1] + q[7]), 0, 0,
                std::cos(dh[8][3])*std::cos(dh[8][1] + q[7]), -std::sin(dh[8][1] + q[7])*std::cos(dh[8][3]), 0, 0,
                std::sin(dh[8][3])*std::cos(dh[8][1] + q[7]), -std::sin(dh[8][3])*std::sin(dh[8][1] + q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 11) << -std::sin(dh[9][1] + q[8]), -std::cos(dh[9][1] + q[8]), 0, 0,
                std::cos(dh[9][3])*std::cos(dh[9][1] + q[8]), -std::sin(dh[9][1] + q[8])*std::cos(dh[9][3]), 0, 0,
                std::sin(dh[9][3])*std::cos(dh[9][1] + q[8]), -std::sin(dh[9][3])*std::sin(dh[9][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(9, 12) << -std::sin(dh[10][1] + q[9]), -std::cos(dh[10][1] + q[9]), 0, 0,
                std::cos(dh[10][3])*std::cos(dh[10][1] + q[9]), -std::sin(dh[10][1] + q[9])*std::cos(dh[10][3]), 0, 0,
                std::sin(dh[10][3])*std::cos(dh[10][1] + q[9]), -std::sin(dh[10][3])*std::sin(dh[10][1] + q[9]), 0, 0,
                0, 0, 0, 0;
    j(10, 15) << -std::sin(dh[11][1] + q[10]), -std::cos(dh[11][1] + q[10]), 0, 0,
                 std::cos(dh[11][3])*std::cos(dh[11][1] + q[10]), -std::sin(dh[11][1] + q[10])*std::cos(dh[11][3]), 0, 0,
                 std::sin(dh[11][3])*std::cos(dh[11][1] + q[10]), -std::sin(dh[11][3])*std::sin(dh[11][1] + q[10]), 0, 0,
                 0, 0, 0, 0;
    j(11, 16) << -std::sin(dh[12][1] + q[11]), -std::cos(dh[12][1] + q[11]), 0, 0,
                 std::cos(dh[12][3])*std::cos(dh[12][1] + q[11]), -std::sin(dh[12][1] + q[11])*std::cos(dh[12][3]), 0, 0,
                 std::sin(dh[12][3])*std::cos(dh[12][1] + q[11]), -std::sin(dh[12][3])*std::sin(dh[12][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    j(12, 17) << -std::sin(dh[13][1] + q[12]), -std::cos(dh[13][1] + q[12]), 0, 0,
                 std::cos(dh[13][3])*std::cos(dh[13][1] + q[12]), -std::sin(dh[13][1] + q[12])*std::cos(dh[13][3]), 0, 0,
                 std::sin(dh[13][3])*std::cos(dh[13][1] + q[12]), -std::sin(dh[13][3])*std::sin(dh[13][1] + q[12]), 0, 0,
                 0, 0, 0, 0;
    j(13, 18) << -std::sin(dh[14][1] + q[13]), -std::cos(dh[14][1] + q[13]), 0, 0,
                 std::cos(dh[14][3])*std::cos(dh[14][1] + q[13]), -std::sin(dh[14][1] + q[13])*std::cos(dh[14][3]), 0, 0,
                 std::sin(dh[14][3])*std::cos(dh[14][1] + q[13]), -std::sin(dh[14][3])*std::sin(dh[14][1] + q[13]), 0, 0,
                 0, 0, 0, 0;
    j(14, 19) << -std::sin(dh[15][1] + q[14]), -std::cos(dh[15][1] + q[14]), 0, 0,
                 std::cos(dh[15][3])*std::cos(dh[15][1] + q[14]), -std::sin(dh[15][1] + q[14])*std::cos(dh[15][3]), 0, 0,
                 std::sin(dh[15][3])*std::cos(dh[15][1] + q[14]), -std::sin(dh[15][3])*std::sin(dh[15][1] + q[14]), 0, 0,
                 0, 0, 0, 0;
    j(15, 20) << -std::sin(dh[16][1] + q[15]), -std::cos(dh[16][1] + q[15]), 0, 0,
                 std::cos(dh[16][3])*std::cos(dh[16][1] + q[15]), -std::sin(dh[16][1] + q[15])*std::cos(dh[16][3]), 0, 0,
                 std::sin(dh[16][3])*std::cos(dh[16][1] + q[15]), -std::sin(dh[16][3])*std::sin(dh[16][1] + q[15]), 0, 0,
                 0, 0, 0, 0;
    j(16, 21) << -std::sin(dh[17][1] + q[16]), -std::cos(dh[17][1] + q[16]), 0, 0,
                 std::cos(dh[17][3])*std::cos(dh[17][1] + q[16]), -std::sin(dh[17][1] + q[16])*std::cos(dh[17][3]), 0, 0,
                 std::sin(dh[17][3])*std::cos(dh[17][1] + q[16]), -std::sin(dh[17][3])*std::sin(dh[17][1] + q[16]), 0, 0,
                 0, 0, 0, 0;
    j(17, 24) << -std::sin(dh[18][1] + q[17]), -std::cos(dh[18][1] + q[17]), 0, 0,
                 std::cos(dh[18][3])*std::cos(dh[18][1] + q[17]), -std::sin(dh[18][1] + q[17])*std::cos(dh[18][3]), 0, 0,
                 std::sin(dh[18][3])*std::cos(dh[18][1] + q[17]), -std::sin(dh[18][3])*std::sin(dh[18][1] + q[17]), 0, 0,
                 0, 0, 0, 0;
    j(18, 25) << -std::sin(dh[19][1] + q[18]), -std::cos(dh[19][1] + q[18]), 0, 0,
                 std::cos(dh[19][3])*std::cos(dh[19][1] + q[18]), -std::sin(dh[19][1] + q[18])*std::cos(dh[19][3]), 0, 0,
                 std::sin(dh[19][3])*std::cos(dh[19][1] + q[18]), -std::sin(dh[19][3])*std::sin(dh[19][1] + q[18]), 0, 0,
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
    j(1, 4) << -std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) - std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]), -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]) + std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               -std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), 0, 0,
               -std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) + std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 3) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), -std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) - std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) + std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) - std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]), -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]) + std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               -std::cos(dh[3][3])*std::cos(-dh[3][1] + q[1] + q[2]), -std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][3]), 0, 0,
               -std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]) + std::sin(dh[3][4])*std::sin(-dh[3][1] + q[1] + q[2]), -std::sin(dh[3][3])*std::sin(-dh[3][1] + q[1] + q[2])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(-dh[3][1] + q[1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 6) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]) - std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]), -std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]) - std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]) + std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 7) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]) - std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]), -std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]) - std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]) + std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 8) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]) - std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]), -std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]) - std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]), 0, 0,
               std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), 0, 0,
               std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]) + std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]), 0, 0,
               0, 0, 0, 0;
    j(6, 9) << std::sin(dh[7][3])*std::sin(dh[7][4])*std::cos(dh[7][1] + q[6]) - std::sin(dh[7][1] + q[6])*std::cos(dh[7][4]), -std::sin(dh[7][3])*std::sin(dh[7][4])*std::sin(dh[7][1] + q[6]) - std::cos(dh[7][4])*std::cos(dh[7][1] + q[6]), 0, 0,
               std::cos(dh[7][3])*std::cos(dh[7][1] + q[6]), -std::sin(dh[7][1] + q[6])*std::cos(dh[7][3]), 0, 0,
               std::sin(dh[7][3])*std::cos(dh[7][4])*std::cos(dh[7][1] + q[6]) + std::sin(dh[7][4])*std::sin(dh[7][1] + q[6]), -std::sin(dh[7][3])*std::sin(dh[7][1] + q[6])*std::cos(dh[7][4]) + std::sin(dh[7][4])*std::cos(dh[7][1] + q[6]), 0, 0,
               0, 0, 0, 0;
    j(7, 10) << std::sin(dh[8][3])*std::sin(dh[8][4])*std::cos(dh[8][1] + q[7]) - std::sin(dh[8][1] + q[7])*std::cos(dh[8][4]), -std::sin(dh[8][3])*std::sin(dh[8][4])*std::sin(dh[8][1] + q[7]) - std::cos(dh[8][4])*std::cos(dh[8][1] + q[7]), 0, 0,
                std::cos(dh[8][3])*std::cos(dh[8][1] + q[7]), -std::sin(dh[8][1] + q[7])*std::cos(dh[8][3]), 0, 0,
                std::sin(dh[8][3])*std::cos(dh[8][4])*std::cos(dh[8][1] + q[7]) + std::sin(dh[8][4])*std::sin(dh[8][1] + q[7]), -std::sin(dh[8][3])*std::sin(dh[8][1] + q[7])*std::cos(dh[8][4]) + std::sin(dh[8][4])*std::cos(dh[8][1] + q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 11) << std::sin(dh[9][3])*std::sin(dh[9][4])*std::cos(dh[9][1] + q[8]) - std::sin(dh[9][1] + q[8])*std::cos(dh[9][4]), -std::sin(dh[9][3])*std::sin(dh[9][4])*std::sin(dh[9][1] + q[8]) - std::cos(dh[9][4])*std::cos(dh[9][1] + q[8]), 0, 0,
                std::cos(dh[9][3])*std::cos(dh[9][1] + q[8]), -std::sin(dh[9][1] + q[8])*std::cos(dh[9][3]), 0, 0,
                std::sin(dh[9][3])*std::cos(dh[9][4])*std::cos(dh[9][1] + q[8]) + std::sin(dh[9][4])*std::sin(dh[9][1] + q[8]), -std::sin(dh[9][3])*std::sin(dh[9][1] + q[8])*std::cos(dh[9][4]) + std::sin(dh[9][4])*std::cos(dh[9][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(9, 12) << std::sin(dh[10][3])*std::sin(dh[10][4])*std::cos(dh[10][1] + q[9]) - std::sin(dh[10][1] + q[9])*std::cos(dh[10][4]), -std::sin(dh[10][3])*std::sin(dh[10][4])*std::sin(dh[10][1] + q[9]) - std::cos(dh[10][4])*std::cos(dh[10][1] + q[9]), 0, 0,
                std::cos(dh[10][3])*std::cos(dh[10][1] + q[9]), -std::sin(dh[10][1] + q[9])*std::cos(dh[10][3]), 0, 0,
                std::sin(dh[10][3])*std::cos(dh[10][4])*std::cos(dh[10][1] + q[9]) + std::sin(dh[10][4])*std::sin(dh[10][1] + q[9]), -std::sin(dh[10][3])*std::sin(dh[10][1] + q[9])*std::cos(dh[10][4]) + std::sin(dh[10][4])*std::cos(dh[10][1] + q[9]), 0, 0,
                0, 0, 0, 0;
    j(10, 15) << std::sin(dh[11][3])*std::sin(dh[11][4])*std::cos(dh[11][1] + q[10]) - std::sin(dh[11][1] + q[10])*std::cos(dh[11][4]), -std::sin(dh[11][3])*std::sin(dh[11][4])*std::sin(dh[11][1] + q[10]) - std::cos(dh[11][4])*std::cos(dh[11][1] + q[10]), 0, 0,
                 std::cos(dh[11][3])*std::cos(dh[11][1] + q[10]), -std::sin(dh[11][1] + q[10])*std::cos(dh[11][3]), 0, 0,
                 std::sin(dh[11][3])*std::cos(dh[11][4])*std::cos(dh[11][1] + q[10]) + std::sin(dh[11][4])*std::sin(dh[11][1] + q[10]), -std::sin(dh[11][3])*std::sin(dh[11][1] + q[10])*std::cos(dh[11][4]) + std::sin(dh[11][4])*std::cos(dh[11][1] + q[10]), 0, 0,
                 0, 0, 0, 0;
    j(11, 16) << std::sin(dh[12][3])*std::sin(dh[12][4])*std::cos(dh[12][1] + q[11]) - std::sin(dh[12][1] + q[11])*std::cos(dh[12][4]), -std::sin(dh[12][3])*std::sin(dh[12][4])*std::sin(dh[12][1] + q[11]) - std::cos(dh[12][4])*std::cos(dh[12][1] + q[11]), 0, 0,
                 std::cos(dh[12][3])*std::cos(dh[12][1] + q[11]), -std::sin(dh[12][1] + q[11])*std::cos(dh[12][3]), 0, 0,
                 std::sin(dh[12][3])*std::cos(dh[12][4])*std::cos(dh[12][1] + q[11]) + std::sin(dh[12][4])*std::sin(dh[12][1] + q[11]), -std::sin(dh[12][3])*std::sin(dh[12][1] + q[11])*std::cos(dh[12][4]) + std::sin(dh[12][4])*std::cos(dh[12][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    j(12, 17) << std::sin(dh[13][3])*std::sin(dh[13][4])*std::cos(dh[13][1] + q[12]) - std::sin(dh[13][1] + q[12])*std::cos(dh[13][4]), -std::sin(dh[13][3])*std::sin(dh[13][4])*std::sin(dh[13][1] + q[12]) - std::cos(dh[13][4])*std::cos(dh[13][1] + q[12]), 0, 0,
                 std::cos(dh[13][3])*std::cos(dh[13][1] + q[12]), -std::sin(dh[13][1] + q[12])*std::cos(dh[13][3]), 0, 0,
                 std::sin(dh[13][3])*std::cos(dh[13][4])*std::cos(dh[13][1] + q[12]) + std::sin(dh[13][4])*std::sin(dh[13][1] + q[12]), -std::sin(dh[13][3])*std::sin(dh[13][1] + q[12])*std::cos(dh[13][4]) + std::sin(dh[13][4])*std::cos(dh[13][1] + q[12]), 0, 0,
                 0, 0, 0, 0;
    j(13, 18) << std::sin(dh[14][3])*std::sin(dh[14][4])*std::cos(dh[14][1] + q[13]) - std::sin(dh[14][1] + q[13])*std::cos(dh[14][4]), -std::sin(dh[14][3])*std::sin(dh[14][4])*std::sin(dh[14][1] + q[13]) - std::cos(dh[14][4])*std::cos(dh[14][1] + q[13]), 0, 0,
                 std::cos(dh[14][3])*std::cos(dh[14][1] + q[13]), -std::sin(dh[14][1] + q[13])*std::cos(dh[14][3]), 0, 0,
                 std::sin(dh[14][3])*std::cos(dh[14][4])*std::cos(dh[14][1] + q[13]) + std::sin(dh[14][4])*std::sin(dh[14][1] + q[13]), -std::sin(dh[14][3])*std::sin(dh[14][1] + q[13])*std::cos(dh[14][4]) + std::sin(dh[14][4])*std::cos(dh[14][1] + q[13]), 0, 0,
                 0, 0, 0, 0;
    j(14, 19) << std::sin(dh[15][3])*std::sin(dh[15][4])*std::cos(dh[15][1] + q[14]) - std::sin(dh[15][1] + q[14])*std::cos(dh[15][4]), -std::sin(dh[15][3])*std::sin(dh[15][4])*std::sin(dh[15][1] + q[14]) - std::cos(dh[15][4])*std::cos(dh[15][1] + q[14]), 0, 0,
                 std::cos(dh[15][3])*std::cos(dh[15][1] + q[14]), -std::sin(dh[15][1] + q[14])*std::cos(dh[15][3]), 0, 0,
                 std::sin(dh[15][3])*std::cos(dh[15][4])*std::cos(dh[15][1] + q[14]) + std::sin(dh[15][4])*std::sin(dh[15][1] + q[14]), -std::sin(dh[15][3])*std::sin(dh[15][1] + q[14])*std::cos(dh[15][4]) + std::sin(dh[15][4])*std::cos(dh[15][1] + q[14]), 0, 0,
                 0, 0, 0, 0;
    j(15, 20) << std::sin(dh[16][3])*std::sin(dh[16][4])*std::cos(dh[16][1] + q[15]) - std::sin(dh[16][1] + q[15])*std::cos(dh[16][4]), -std::sin(dh[16][3])*std::sin(dh[16][4])*std::sin(dh[16][1] + q[15]) - std::cos(dh[16][4])*std::cos(dh[16][1] + q[15]), 0, 0,
                 std::cos(dh[16][3])*std::cos(dh[16][1] + q[15]), -std::sin(dh[16][1] + q[15])*std::cos(dh[16][3]), 0, 0,
                 std::sin(dh[16][3])*std::cos(dh[16][4])*std::cos(dh[16][1] + q[15]) + std::sin(dh[16][4])*std::sin(dh[16][1] + q[15]), -std::sin(dh[16][3])*std::sin(dh[16][1] + q[15])*std::cos(dh[16][4]) + std::sin(dh[16][4])*std::cos(dh[16][1] + q[15]), 0, 0,
                 0, 0, 0, 0;
    j(16, 21) << std::sin(dh[17][3])*std::sin(dh[17][4])*std::cos(dh[17][1] + q[16]) - std::sin(dh[17][1] + q[16])*std::cos(dh[17][4]), -std::sin(dh[17][3])*std::sin(dh[17][4])*std::sin(dh[17][1] + q[16]) - std::cos(dh[17][4])*std::cos(dh[17][1] + q[16]), 0, 0,
                 std::cos(dh[17][3])*std::cos(dh[17][1] + q[16]), -std::sin(dh[17][1] + q[16])*std::cos(dh[17][3]), 0, 0,
                 std::sin(dh[17][3])*std::cos(dh[17][4])*std::cos(dh[17][1] + q[16]) + std::sin(dh[17][4])*std::sin(dh[17][1] + q[16]), -std::sin(dh[17][3])*std::sin(dh[17][1] + q[16])*std::cos(dh[17][4]) + std::sin(dh[17][4])*std::cos(dh[17][1] + q[16]), 0, 0,
                 0, 0, 0, 0;
    j(17, 24) << std::sin(dh[18][3])*std::sin(dh[18][4])*std::cos(dh[18][1] + q[17]) - std::sin(dh[18][1] + q[17])*std::cos(dh[18][4]), -std::sin(dh[18][3])*std::sin(dh[18][4])*std::sin(dh[18][1] + q[17]) - std::cos(dh[18][4])*std::cos(dh[18][1] + q[17]), 0, 0,
                 std::cos(dh[18][3])*std::cos(dh[18][1] + q[17]), -std::sin(dh[18][1] + q[17])*std::cos(dh[18][3]), 0, 0,
                 std::sin(dh[18][3])*std::cos(dh[18][4])*std::cos(dh[18][1] + q[17]) + std::sin(dh[18][4])*std::sin(dh[18][1] + q[17]), -std::sin(dh[18][3])*std::sin(dh[18][1] + q[17])*std::cos(dh[18][4]) + std::sin(dh[18][4])*std::cos(dh[18][1] + q[17]), 0, 0,
                 0, 0, 0, 0;
    j(18, 25) << std::sin(dh[19][3])*std::sin(dh[19][4])*std::cos(dh[19][1] + q[18]) - std::sin(dh[19][1] + q[18])*std::cos(dh[19][4]), -std::sin(dh[19][3])*std::sin(dh[19][4])*std::sin(dh[19][1] + q[18]) - std::cos(dh[19][4])*std::cos(dh[19][1] + q[18]), 0, 0,
                 std::cos(dh[19][3])*std::cos(dh[19][1] + q[18]), -std::sin(dh[19][1] + q[18])*std::cos(dh[19][3]), 0, 0,
                 std::sin(dh[19][3])*std::cos(dh[19][4])*std::cos(dh[19][1] + q[18]) + std::sin(dh[19][4])*std::sin(dh[19][1] + q[18]), -std::sin(dh[19][3])*std::sin(dh[19][1] + q[18])*std::cos(dh[19][4]) + std::sin(dh[19][4])*std::cos(dh[19][1] + q[18]), 0, 0,
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
    f(8) = f(7) * f(8);
    f(9) = f(8) * f(9);
    f(10) = f(9) * f(10);
    f(11) = f(10) * f(11);
    f(12) = f(11) * f(12);
    f(13) = f(12) * f(13);
    f(14) = f(4) * f(14);
    f(15) = f(14) * f(15);
    f(16) = f(15) * f(16);
    f(17) = f(16) * f(17);
    f(18) = f(17) * f(18);
    f(19) = f(18) * f(19);
    f(20) = f(19) * f(20);
    f(21) = f(20) * f(21);
    f(22) = f(21) * f(22);
    f(23) = f(4) * f(23);
    f(24) = f(23) * f(24);
    f(25) = f(24) * f(25);
    f(26) = f(25) * f(26);
    
}


void combine_jacs(JACS& j, DICT& d){
    // 0
    j(0, 1) = d(0, 0) * j(0, 1);
    j(0, 2) = j(0, 1) * d(2, 2);
    j(0, 3) = j(0, 1) * d(2, 3);
    j(0, 4) = j(0, 1) * d(2, 4);
    j(0, 5) = j(0, 1) * d(2, 5);
    j(0, 6) = j(0, 1) * d(2, 6);
    j(0, 7) = j(0, 1) * d(2, 7);
    j(0, 8) = j(0, 1) * d(2, 8);
    j(0, 9) = j(0, 1) * d(2, 9);
    j(0, 10) = j(0, 1) * d(2, 10);
    j(0, 11) = j(0, 1) * d(2, 11);
    j(0, 12) = j(0, 1) * d(2, 12);
    j(0, 13) = j(0, 1) * d(2, 13);
    j(0, 14) = j(0, 1) * d(2, 14);
    j(0, 15) = j(0, 1) * d(2, 15);
    j(0, 16) = j(0, 1) * d(2, 16);
    j(0, 17) = j(0, 1) * d(2, 17);
    j(0, 18) = j(0, 1) * d(2, 18);
    j(0, 19) = j(0, 1) * d(2, 19);
    j(0, 20) = j(0, 1) * d(2, 20);
    j(0, 21) = j(0, 1) * d(2, 21);
    j(0, 22) = j(0, 1) * d(2, 22);
    j(0, 23) = j(0, 1) * d(2, 23);
    j(0, 24) = j(0, 1) * d(2, 24);
    j(0, 25) = j(0, 1) * d(2, 25);
    j(0, 26) = j(0, 1) * d(2, 26);
    // 1
    j(1, 2) = d(0, 1) * j(1, 2);
    j(1, 3) = j(1, 2) * d(3, 3);
    j(1, 4) = j(1, 3) * d(4, 4) + d(0, 3) * j(1, 4);
    j(1, 5) = j(1, 4) * d(5, 5);
    j(1, 6) = j(1, 4) * d(5, 6);
    j(1, 7) = j(1, 4) * d(5, 7);
    j(1, 8) = j(1, 4) * d(5, 8);
    j(1, 9) = j(1, 4) * d(5, 9);
    j(1, 10) = j(1, 4) * d(5, 10);
    j(1, 11) = j(1, 4) * d(5, 11);
    j(1, 12) = j(1, 4) * d(5, 12);
    j(1, 13) = j(1, 4) * d(5, 13);
    j(1, 14) = j(1, 4) * d(14, 14);
    j(1, 15) = j(1, 4) * d(14, 15);
    j(1, 16) = j(1, 4) * d(14, 16);
    j(1, 17) = j(1, 4) * d(14, 17);
    j(1, 18) = j(1, 4) * d(14, 18);
    j(1, 19) = j(1, 4) * d(14, 19);
    j(1, 20) = j(1, 4) * d(14, 20);
    j(1, 21) = j(1, 4) * d(14, 21);
    j(1, 22) = j(1, 4) * d(14, 22);
    j(1, 23) = j(1, 4) * d(23, 23);
    j(1, 24) = j(1, 4) * d(23, 24);
    j(1, 25) = j(1, 4) * d(23, 25);
    j(1, 26) = j(1, 4) * d(23, 26);
    // 2
    j(2, 3) = d(0, 2) * j(2, 3);
    j(2, 4) = j(2, 3) * d(4, 4) + d(0, 3) * j(2, 4);
    j(2, 5) = j(2, 4) * d(5, 5);
    j(2, 6) = j(2, 4) * d(5, 6);
    j(2, 7) = j(2, 4) * d(5, 7);
    j(2, 8) = j(2, 4) * d(5, 8);
    j(2, 9) = j(2, 4) * d(5, 9);
    j(2, 10) = j(2, 4) * d(5, 10);
    j(2, 11) = j(2, 4) * d(5, 11);
    j(2, 12) = j(2, 4) * d(5, 12);
    j(2, 13) = j(2, 4) * d(5, 13);
    j(2, 14) = j(2, 4) * d(14, 14);
    j(2, 15) = j(2, 4) * d(14, 15);
    j(2, 16) = j(2, 4) * d(14, 16);
    j(2, 17) = j(2, 4) * d(14, 17);
    j(2, 18) = j(2, 4) * d(14, 18);
    j(2, 19) = j(2, 4) * d(14, 19);
    j(2, 20) = j(2, 4) * d(14, 20);
    j(2, 21) = j(2, 4) * d(14, 21);
    j(2, 22) = j(2, 4) * d(14, 22);
    j(2, 23) = j(2, 4) * d(23, 23);
    j(2, 24) = j(2, 4) * d(23, 24);
    j(2, 25) = j(2, 4) * d(23, 25);
    j(2, 26) = j(2, 4) * d(23, 26);
    // 3
    j(3, 6) = d(0, 5) * j(3, 6);
    j(3, 7) = j(3, 6) * d(7, 7);
    j(3, 8) = j(3, 6) * d(7, 8);
    j(3, 9) = j(3, 6) * d(7, 9);
    j(3, 10) = j(3, 6) * d(7, 10);
    j(3, 11) = j(3, 6) * d(7, 11);
    j(3, 12) = j(3, 6) * d(7, 12);
    j(3, 13) = j(3, 6) * d(7, 13);
    // 4
    j(4, 7) = d(0, 6) * j(4, 7);
    j(4, 8) = j(4, 7) * d(8, 8);
    j(4, 9) = j(4, 7) * d(8, 9);
    j(4, 10) = j(4, 7) * d(8, 10);
    j(4, 11) = j(4, 7) * d(8, 11);
    j(4, 12) = j(4, 7) * d(8, 12);
    j(4, 13) = j(4, 7) * d(8, 13);
    // 5
    j(5, 8) = d(0, 7) * j(5, 8);
    j(5, 9) = j(5, 8) * d(9, 9);
    j(5, 10) = j(5, 8) * d(9, 10);
    j(5, 11) = j(5, 8) * d(9, 11);
    j(5, 12) = j(5, 8) * d(9, 12);
    j(5, 13) = j(5, 8) * d(9, 13);
    // 6
    j(6, 9) = d(0, 8) * j(6, 9);
    j(6, 10) = j(6, 9) * d(10, 10);
    j(6, 11) = j(6, 9) * d(10, 11);
    j(6, 12) = j(6, 9) * d(10, 12);
    j(6, 13) = j(6, 9) * d(10, 13);
    // 7
    j(7, 10) = d(0, 9) * j(7, 10);
    j(7, 11) = j(7, 10) * d(11, 11);
    j(7, 12) = j(7, 10) * d(11, 12);
    j(7, 13) = j(7, 10) * d(11, 13);
    // 8
    j(8, 11) = d(0, 10) * j(8, 11);
    j(8, 12) = j(8, 11) * d(12, 12);
    j(8, 13) = j(8, 11) * d(12, 13);
    // 9
    j(9, 12) = d(0, 11) * j(9, 12);
    j(9, 13) = j(9, 12) * d(13, 13);
    // 10
    j(10, 15) = d(0, 14) * j(10, 15);
    j(10, 16) = j(10, 15) * d(16, 16);
    j(10, 17) = j(10, 15) * d(16, 17);
    j(10, 18) = j(10, 15) * d(16, 18);
    j(10, 19) = j(10, 15) * d(16, 19);
    j(10, 20) = j(10, 15) * d(16, 20);
    j(10, 21) = j(10, 15) * d(16, 21);
    j(10, 22) = j(10, 15) * d(16, 22);
    // 11
    j(11, 16) = d(0, 15) * j(11, 16);
    j(11, 17) = j(11, 16) * d(17, 17);
    j(11, 18) = j(11, 16) * d(17, 18);
    j(11, 19) = j(11, 16) * d(17, 19);
    j(11, 20) = j(11, 16) * d(17, 20);
    j(11, 21) = j(11, 16) * d(17, 21);
    j(11, 22) = j(11, 16) * d(17, 22);
    // 12
    j(12, 17) = d(0, 16) * j(12, 17);
    j(12, 18) = j(12, 17) * d(18, 18);
    j(12, 19) = j(12, 17) * d(18, 19);
    j(12, 20) = j(12, 17) * d(18, 20);
    j(12, 21) = j(12, 17) * d(18, 21);
    j(12, 22) = j(12, 17) * d(18, 22);
    // 13
    j(13, 18) = d(0, 17) * j(13, 18);
    j(13, 19) = j(13, 18) * d(19, 19);
    j(13, 20) = j(13, 18) * d(19, 20);
    j(13, 21) = j(13, 18) * d(19, 21);
    j(13, 22) = j(13, 18) * d(19, 22);
    // 14
    j(14, 19) = d(0, 18) * j(14, 19);
    j(14, 20) = j(14, 19) * d(20, 20);
    j(14, 21) = j(14, 19) * d(20, 21);
    j(14, 22) = j(14, 19) * d(20, 22);
    // 15
    j(15, 20) = d(0, 19) * j(15, 20);
    j(15, 21) = j(15, 20) * d(21, 21);
    j(15, 22) = j(15, 20) * d(21, 22);
    // 16
    j(16, 21) = d(0, 20) * j(16, 21);
    j(16, 22) = j(16, 21) * d(22, 22);
    // 17
    j(17, 24) = d(0, 23) * j(17, 24);
    j(17, 25) = j(17, 24) * d(25, 25);
    j(17, 26) = j(17, 24) * d(25, 26);
    // 18
    j(18, 25) = d(0, 24) * j(18, 25);
    j(18, 26) = j(18, 25) * d(26, 26);
    
}


void combine_dict(FRAMES& f, DICT& d){
    // 26
    d(26, 26) = f(26);
    // 25
    d(25, 25) = f(25);
    d(25, 26) = f(25) * d(26, 26);
    // 24
    d(24, 24) = f(24);
    d(24, 25) = f(24) * d(25, 25);
    d(24, 26) = f(24) * d(25, 26);
    // 23
    d(23, 23) = f(23);
    d(23, 24) = f(23) * d(24, 24);
    d(23, 25) = f(23) * d(24, 25);
    d(23, 26) = f(23) * d(24, 26);
    // 22
    d(22, 22) = f(22);
    // 21
    d(21, 21) = f(21);
    d(21, 22) = f(21) * d(22, 22);
    // 20
    d(20, 20) = f(20);
    d(20, 21) = f(20) * d(21, 21);
    d(20, 22) = f(20) * d(21, 22);
    // 19
    d(19, 19) = f(19);
    d(19, 20) = f(19) * d(20, 20);
    d(19, 21) = f(19) * d(20, 21);
    d(19, 22) = f(19) * d(20, 22);
    // 18
    d(18, 18) = f(18);
    d(18, 19) = f(18) * d(19, 19);
    d(18, 20) = f(18) * d(19, 20);
    d(18, 21) = f(18) * d(19, 21);
    d(18, 22) = f(18) * d(19, 22);
    // 17
    d(17, 17) = f(17);
    d(17, 18) = f(17) * d(18, 18);
    d(17, 19) = f(17) * d(18, 19);
    d(17, 20) = f(17) * d(18, 20);
    d(17, 21) = f(17) * d(18, 21);
    d(17, 22) = f(17) * d(18, 22);
    // 16
    d(16, 16) = f(16);
    d(16, 17) = f(16) * d(17, 17);
    d(16, 18) = f(16) * d(17, 18);
    d(16, 19) = f(16) * d(17, 19);
    d(16, 20) = f(16) * d(17, 20);
    d(16, 21) = f(16) * d(17, 21);
    d(16, 22) = f(16) * d(17, 22);
    // 15
    d(15, 15) = f(15);
    d(15, 16) = f(15) * d(16, 16);
    d(15, 17) = f(15) * d(16, 17);
    d(15, 18) = f(15) * d(16, 18);
    d(15, 19) = f(15) * d(16, 19);
    d(15, 20) = f(15) * d(16, 20);
    d(15, 21) = f(15) * d(16, 21);
    d(15, 22) = f(15) * d(16, 22);
    // 14
    d(14, 14) = f(14);
    d(14, 15) = f(14) * d(15, 15);
    d(14, 16) = f(14) * d(15, 16);
    d(14, 17) = f(14) * d(15, 17);
    d(14, 18) = f(14) * d(15, 18);
    d(14, 19) = f(14) * d(15, 19);
    d(14, 20) = f(14) * d(15, 20);
    d(14, 21) = f(14) * d(15, 21);
    d(14, 22) = f(14) * d(15, 22);
    // 13
    d(13, 13) = f(13);
    // 12
    d(12, 12) = f(12);
    d(12, 13) = f(12) * d(13, 13);
    // 11
    d(11, 11) = f(11);
    d(11, 12) = f(11) * d(12, 12);
    d(11, 13) = f(11) * d(12, 13);
    // 10
    d(10, 10) = f(10);
    d(10, 11) = f(10) * d(11, 11);
    d(10, 12) = f(10) * d(11, 12);
    d(10, 13) = f(10) * d(11, 13);
    // 9
    d(9, 9) = f(9);
    d(9, 10) = f(9) * d(10, 10);
    d(9, 11) = f(9) * d(10, 11);
    d(9, 12) = f(9) * d(10, 12);
    d(9, 13) = f(9) * d(10, 13);
    // 8
    d(8, 8) = f(8);
    d(8, 9) = f(8) * d(9, 9);
    d(8, 10) = f(8) * d(9, 10);
    d(8, 11) = f(8) * d(9, 11);
    d(8, 12) = f(8) * d(9, 12);
    d(8, 13) = f(8) * d(9, 13);
    // 7
    d(7, 7) = f(7);
    d(7, 8) = f(7) * d(8, 8);
    d(7, 9) = f(7) * d(8, 9);
    d(7, 10) = f(7) * d(8, 10);
    d(7, 11) = f(7) * d(8, 11);
    d(7, 12) = f(7) * d(8, 12);
    d(7, 13) = f(7) * d(8, 13);
    // 6
    d(6, 6) = f(6);
    d(6, 7) = f(6) * d(7, 7);
    d(6, 8) = f(6) * d(7, 8);
    d(6, 9) = f(6) * d(7, 9);
    d(6, 10) = f(6) * d(7, 10);
    d(6, 11) = f(6) * d(7, 11);
    d(6, 12) = f(6) * d(7, 12);
    d(6, 13) = f(6) * d(7, 13);
    // 5
    d(5, 5) = f(5);
    d(5, 6) = f(5) * d(6, 6);
    d(5, 7) = f(5) * d(6, 7);
    d(5, 8) = f(5) * d(6, 8);
    d(5, 9) = f(5) * d(6, 9);
    d(5, 10) = f(5) * d(6, 10);
    d(5, 11) = f(5) * d(6, 11);
    d(5, 12) = f(5) * d(6, 12);
    d(5, 13) = f(5) * d(6, 13);
    // 4
    d(4, 4) = f(4);
    d(4, 5) = f(4) * d(5, 5);
    d(4, 6) = f(4) * d(5, 6);
    d(4, 7) = f(4) * d(5, 7);
    d(4, 8) = f(4) * d(5, 8);
    d(4, 9) = f(4) * d(5, 9);
    d(4, 10) = f(4) * d(5, 10);
    d(4, 11) = f(4) * d(5, 11);
    d(4, 12) = f(4) * d(5, 12);
    d(4, 13) = f(4) * d(5, 13);
    d(4, 14) = f(4) * d(14, 14);
    d(4, 15) = f(4) * d(14, 15);
    d(4, 16) = f(4) * d(14, 16);
    d(4, 17) = f(4) * d(14, 17);
    d(4, 18) = f(4) * d(14, 18);
    d(4, 19) = f(4) * d(14, 19);
    d(4, 20) = f(4) * d(14, 20);
    d(4, 21) = f(4) * d(14, 21);
    d(4, 22) = f(4) * d(14, 22);
    d(4, 23) = f(4) * d(23, 23);
    d(4, 24) = f(4) * d(23, 24);
    d(4, 25) = f(4) * d(23, 25);
    d(4, 26) = f(4) * d(23, 26);
    // 3
    d(3, 3) = f(3);
    d(3, 4) = f(3) * d(4, 4);
    d(3, 5) = f(3) * d(4, 5);
    d(3, 6) = f(3) * d(4, 6);
    d(3, 7) = f(3) * d(4, 7);
    d(3, 8) = f(3) * d(4, 8);
    d(3, 9) = f(3) * d(4, 9);
    d(3, 10) = f(3) * d(4, 10);
    d(3, 11) = f(3) * d(4, 11);
    d(3, 12) = f(3) * d(4, 12);
    d(3, 13) = f(3) * d(4, 13);
    d(3, 14) = f(3) * d(4, 14);
    d(3, 15) = f(3) * d(4, 15);
    d(3, 16) = f(3) * d(4, 16);
    d(3, 17) = f(3) * d(4, 17);
    d(3, 18) = f(3) * d(4, 18);
    d(3, 19) = f(3) * d(4, 19);
    d(3, 20) = f(3) * d(4, 20);
    d(3, 21) = f(3) * d(4, 21);
    d(3, 22) = f(3) * d(4, 22);
    d(3, 23) = f(3) * d(4, 23);
    d(3, 24) = f(3) * d(4, 24);
    d(3, 25) = f(3) * d(4, 25);
    d(3, 26) = f(3) * d(4, 26);
    // 2
    d(2, 2) = f(2);
    d(2, 3) = f(2) * d(3, 3);
    d(2, 4) = f(2) * d(3, 4);
    d(2, 5) = f(2) * d(3, 5);
    d(2, 6) = f(2) * d(3, 6);
    d(2, 7) = f(2) * d(3, 7);
    d(2, 8) = f(2) * d(3, 8);
    d(2, 9) = f(2) * d(3, 9);
    d(2, 10) = f(2) * d(3, 10);
    d(2, 11) = f(2) * d(3, 11);
    d(2, 12) = f(2) * d(3, 12);
    d(2, 13) = f(2) * d(3, 13);
    d(2, 14) = f(2) * d(3, 14);
    d(2, 15) = f(2) * d(3, 15);
    d(2, 16) = f(2) * d(3, 16);
    d(2, 17) = f(2) * d(3, 17);
    d(2, 18) = f(2) * d(3, 18);
    d(2, 19) = f(2) * d(3, 19);
    d(2, 20) = f(2) * d(3, 20);
    d(2, 21) = f(2) * d(3, 21);
    d(2, 22) = f(2) * d(3, 22);
    d(2, 23) = f(2) * d(3, 23);
    d(2, 24) = f(2) * d(3, 24);
    d(2, 25) = f(2) * d(3, 25);
    d(2, 26) = f(2) * d(3, 26);
    // 1
    d(1, 1) = f(1);
    d(1, 2) = f(1) * d(2, 2);
    d(1, 3) = f(1) * d(2, 3);
    d(1, 4) = f(1) * d(2, 4);
    d(1, 5) = f(1) * d(2, 5);
    d(1, 6) = f(1) * d(2, 6);
    d(1, 7) = f(1) * d(2, 7);
    d(1, 8) = f(1) * d(2, 8);
    d(1, 9) = f(1) * d(2, 9);
    d(1, 10) = f(1) * d(2, 10);
    d(1, 11) = f(1) * d(2, 11);
    d(1, 12) = f(1) * d(2, 12);
    d(1, 13) = f(1) * d(2, 13);
    d(1, 14) = f(1) * d(2, 14);
    d(1, 15) = f(1) * d(2, 15);
    d(1, 16) = f(1) * d(2, 16);
    d(1, 17) = f(1) * d(2, 17);
    d(1, 18) = f(1) * d(2, 18);
    d(1, 19) = f(1) * d(2, 19);
    d(1, 20) = f(1) * d(2, 20);
    d(1, 21) = f(1) * d(2, 21);
    d(1, 22) = f(1) * d(2, 22);
    d(1, 23) = f(1) * d(2, 23);
    d(1, 24) = f(1) * d(2, 24);
    d(1, 25) = f(1) * d(2, 25);
    d(1, 26) = f(1) * d(2, 26);
    // 0
    d(0, 0) = f(0);
    d(0, 1) = f(0) * d(1, 1);
    d(0, 2) = f(0) * d(1, 2);
    d(0, 3) = f(0) * d(1, 3);
    d(0, 4) = f(0) * d(1, 4);
    d(0, 5) = f(0) * d(1, 5);
    d(0, 6) = f(0) * d(1, 6);
    d(0, 7) = f(0) * d(1, 7);
    d(0, 8) = f(0) * d(1, 8);
    d(0, 9) = f(0) * d(1, 9);
    d(0, 10) = f(0) * d(1, 10);
    d(0, 11) = f(0) * d(1, 11);
    d(0, 12) = f(0) * d(1, 12);
    d(0, 13) = f(0) * d(1, 13);
    d(0, 14) = f(0) * d(1, 14);
    d(0, 15) = f(0) * d(1, 15);
    d(0, 16) = f(0) * d(1, 16);
    d(0, 17) = f(0) * d(1, 17);
    d(0, 18) = f(0) * d(1, 18);
    d(0, 19) = f(0) * d(1, 19);
    d(0, 20) = f(0) * d(1, 20);
    d(0, 21) = f(0) * d(1, 21);
    d(0, 22) = f(0) * d(1, 22);
    d(0, 23) = f(0) * d(1, 23);
    d(0, 24) = f(0) * d(1, 24);
    d(0, 25) = f(0) * d(1, 25);
    d(0, 26) = f(0) * d(1, 26);
    
}

    