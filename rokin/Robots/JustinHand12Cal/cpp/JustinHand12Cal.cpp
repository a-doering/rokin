#include "JustinHand12Cal.h"


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
    f(1) << -0.140413880, 0.6964277, 0.703755920, -0.0087530,
            -0.168684120, -0.717232940, 0.676108410, -0.04568244,
            0.975617550, -0.02377744, 0.218185760, 0.124312450,
            0, 0, 0, 1.;
    f(2) << std::cos(q[0]), -std::sin(q[0]), 0, 0,
            std::sin(q[0]), std::cos(q[0]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(3) << std::cos(q[1]), -std::sin(q[1]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[1]), std::cos(q[1]), 0, 0,
            0, 0, 0, 1;
    f(4) << std::cos(q[2]), -std::sin(q[2]), 0, 0.0750,
            std::sin(q[2]), std::cos(q[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(5) << std::cos(q[2] - 0.5*pi), std::cos(q[2]), 0, 0.04,
            std::sin(q[2] - 0.5*pi), std::cos(q[2] - 0.5*pi), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(6) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    f(7) << -0.181930170, 0.02798179, 0.982913240, -0.05103251,
            -0.197346280, -0.980295940, -0.0086201, 0.0029209,
            0.963304650, -0.195542530, 0.183867510, 0.1292965,
            0, 0, 0, 1.;
    f(8) << std::cos(q[3]), -std::sin(q[3]), 0, 0,
            std::sin(q[3]), std::cos(q[3]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    f(9) << std::cos(q[4]), -std::sin(q[4]), 0, 0,
            0, 0, -1, 0,
            std::sin(q[4]), std::cos(q[4]), 0, 0,
            0, 0, 0, 1;
    f(10) << std::cos(q[5]), -std::sin(q[5]), 0, 0.0750,
             std::sin(q[5]), std::cos(q[5]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(11) << std::cos(q[5] - 0.5*pi), std::cos(q[5]), 0, 0.04,
             std::sin(q[5] - 0.5*pi), std::cos(q[5] - 0.5*pi), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(12) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(13) << -0.210028860, 0.05765612, 0.975993670, -0.04598232,
             -0.09710584, -0.994553860, 0.03785587, 0.02769158,
             0.9728609, -0.08682386, 0.214483760, 0.138843570,
             0, 0, 0, 1.;
    f(14) << std::cos(q[6]), -std::sin(q[6]), 0, 0,
             std::sin(q[6]), std::cos(q[6]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(15) << std::cos(q[7]), -std::sin(q[7]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[7]), std::cos(q[7]), 0, 0,
             0, 0, 0, 1;
    f(16) << std::cos(q[8]), -std::sin(q[8]), 0, 0.0750,
             std::sin(q[8]), std::cos(q[8]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(17) << std::cos(q[8] - 0.5*pi), std::cos(q[8]), 0, 0.04,
             std::sin(q[8] - 0.5*pi), std::cos(q[8] - 0.5*pi), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(18) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(19) << 0.430352050, 0.122160610, -0.8943567, 0.05180671,
             -0.337389210, 0.9407565, -0.03384864, 0.03071566,
             0.8372369, 0.316313130, 0.446072160, 0.102831510,
             0, 0, 0, 1.;
    f(20) << std::cos(q[9]), -std::sin(q[9]), 0, 0,
             std::sin(q[9]), std::cos(q[9]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(21) << std::cos(q[10]), -std::sin(q[10]), 0, 0,
             0, 0, -1, 0,
             std::sin(q[10]), std::cos(q[10]), 0, 0,
             0, 0, 0, 1;
    f(22) << std::cos(q[11]), -std::sin(q[11]), 0, 0.0750,
             std::sin(q[11]), std::cos(q[11]), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(23) << std::cos(q[11] - 0.5*pi), std::cos(q[11]), 0, 0.04,
             std::sin(q[11] - 0.5*pi), std::cos(q[11] - 0.5*pi), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    f(24) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void fill_frames_dh4(FRAMES& f, JOINTS& q, DH4& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0,
            0, 0, 0, 1.;
    f(1) << -0.140413880, 0.6964277, 0.703755920, -0.0087530,
            -0.168684120, -0.717232940, 0.676108410, -0.04568244,
            0.975617550, -0.02377744, 0.218185760, 0.124312450,
            0, 0, 0, 1.;
    f(2) << std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0]), 0, dh[0][2],
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), std::cos(dh[0][3]), dh[0][0]*std::cos(dh[0][3]),
            0, 0, 0, 1;
    f(3) << std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1]), 0, dh[1][2],
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), std::cos(dh[1][3]), dh[1][0]*std::cos(dh[1][3]),
            0, 0, 0, 1;
    f(4) << std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2]), 0, dh[2][2],
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), std::cos(dh[2][3]), dh[2][0]*std::cos(dh[2][3]),
            0, 0, 0, 1;
    f(5) << std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2]), 0, dh[3][2],
            std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[2]), std::sin(dh[3][3])*std::cos(dh[3][1] + q[2]), std::cos(dh[3][3]), dh[3][0]*std::cos(dh[3][3]),
            0, 0, 0, 1;
    f(6) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    f(7) << -0.181930170, 0.02798179, 0.982913240, -0.05103251,
            -0.197346280, -0.980295940, -0.0086201, 0.0029209,
            0.963304650, -0.195542530, 0.183867510, 0.1292965,
            0, 0, 0, 1.;
    f(8) << std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3]), 0, dh[4][2],
            std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[3]), std::sin(dh[4][3])*std::cos(dh[4][1] + q[3]), std::cos(dh[4][3]), dh[4][0]*std::cos(dh[4][3]),
            0, 0, 0, 1;
    f(9) << std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4]), 0, dh[5][2],
            std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[4]), std::sin(dh[5][3])*std::cos(dh[5][1] + q[4]), std::cos(dh[5][3]), dh[5][0]*std::cos(dh[5][3]),
            0, 0, 0, 1;
    f(10) << std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5]), 0, dh[6][2],
             std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
             std::sin(dh[6][3])*std::sin(dh[6][1] + q[5]), std::sin(dh[6][3])*std::cos(dh[6][1] + q[5]), std::cos(dh[6][3]), dh[6][0]*std::cos(dh[6][3]),
             0, 0, 0, 1;
    f(11) << std::cos(dh[7][1] + q[5]), -std::sin(dh[7][1] + q[5]), 0, dh[7][2],
             std::sin(dh[7][1] + q[5])*std::cos(dh[7][3]), std::cos(dh[7][3])*std::cos(dh[7][1] + q[5]), -std::sin(dh[7][3]), -dh[7][0]*std::sin(dh[7][3]),
             std::sin(dh[7][3])*std::sin(dh[7][1] + q[5]), std::sin(dh[7][3])*std::cos(dh[7][1] + q[5]), std::cos(dh[7][3]), dh[7][0]*std::cos(dh[7][3]),
             0, 0, 0, 1;
    f(12) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(13) << -0.210028860, 0.05765612, 0.975993670, -0.04598232,
             -0.09710584, -0.994553860, 0.03785587, 0.02769158,
             0.9728609, -0.08682386, 0.214483760, 0.138843570,
             0, 0, 0, 1.;
    f(14) << std::cos(dh[8][1] + q[6]), -std::sin(dh[8][1] + q[6]), 0, dh[8][2],
             std::sin(dh[8][1] + q[6])*std::cos(dh[8][3]), std::cos(dh[8][3])*std::cos(dh[8][1] + q[6]), -std::sin(dh[8][3]), -dh[8][0]*std::sin(dh[8][3]),
             std::sin(dh[8][3])*std::sin(dh[8][1] + q[6]), std::sin(dh[8][3])*std::cos(dh[8][1] + q[6]), std::cos(dh[8][3]), dh[8][0]*std::cos(dh[8][3]),
             0, 0, 0, 1;
    f(15) << std::cos(dh[9][1] + q[7]), -std::sin(dh[9][1] + q[7]), 0, dh[9][2],
             std::sin(dh[9][1] + q[7])*std::cos(dh[9][3]), std::cos(dh[9][3])*std::cos(dh[9][1] + q[7]), -std::sin(dh[9][3]), -dh[9][0]*std::sin(dh[9][3]),
             std::sin(dh[9][3])*std::sin(dh[9][1] + q[7]), std::sin(dh[9][3])*std::cos(dh[9][1] + q[7]), std::cos(dh[9][3]), dh[9][0]*std::cos(dh[9][3]),
             0, 0, 0, 1;
    f(16) << std::cos(dh[10][1] + q[8]), -std::sin(dh[10][1] + q[8]), 0, dh[10][2],
             std::sin(dh[10][1] + q[8])*std::cos(dh[10][3]), std::cos(dh[10][3])*std::cos(dh[10][1] + q[8]), -std::sin(dh[10][3]), -dh[10][0]*std::sin(dh[10][3]),
             std::sin(dh[10][3])*std::sin(dh[10][1] + q[8]), std::sin(dh[10][3])*std::cos(dh[10][1] + q[8]), std::cos(dh[10][3]), dh[10][0]*std::cos(dh[10][3]),
             0, 0, 0, 1;
    f(17) << std::cos(dh[11][1] + q[8]), -std::sin(dh[11][1] + q[8]), 0, dh[11][2],
             std::sin(dh[11][1] + q[8])*std::cos(dh[11][3]), std::cos(dh[11][3])*std::cos(dh[11][1] + q[8]), -std::sin(dh[11][3]), -dh[11][0]*std::sin(dh[11][3]),
             std::sin(dh[11][3])*std::sin(dh[11][1] + q[8]), std::sin(dh[11][3])*std::cos(dh[11][1] + q[8]), std::cos(dh[11][3]), dh[11][0]*std::cos(dh[11][3]),
             0, 0, 0, 1;
    f(18) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(19) << 0.430352050, 0.122160610, -0.8943567, 0.05180671,
             -0.337389210, 0.9407565, -0.03384864, 0.03071566,
             0.8372369, 0.316313130, 0.446072160, 0.102831510,
             0, 0, 0, 1.;
    f(20) << std::cos(dh[12][1] + q[9]), -std::sin(dh[12][1] + q[9]), 0, dh[12][2],
             std::sin(dh[12][1] + q[9])*std::cos(dh[12][3]), std::cos(dh[12][3])*std::cos(dh[12][1] + q[9]), -std::sin(dh[12][3]), -dh[12][0]*std::sin(dh[12][3]),
             std::sin(dh[12][3])*std::sin(dh[12][1] + q[9]), std::sin(dh[12][3])*std::cos(dh[12][1] + q[9]), std::cos(dh[12][3]), dh[12][0]*std::cos(dh[12][3]),
             0, 0, 0, 1;
    f(21) << std::cos(dh[13][1] + q[10]), -std::sin(dh[13][1] + q[10]), 0, dh[13][2],
             std::sin(dh[13][1] + q[10])*std::cos(dh[13][3]), std::cos(dh[13][3])*std::cos(dh[13][1] + q[10]), -std::sin(dh[13][3]), -dh[13][0]*std::sin(dh[13][3]),
             std::sin(dh[13][3])*std::sin(dh[13][1] + q[10]), std::sin(dh[13][3])*std::cos(dh[13][1] + q[10]), std::cos(dh[13][3]), dh[13][0]*std::cos(dh[13][3]),
             0, 0, 0, 1;
    f(22) << std::cos(dh[14][1] + q[11]), -std::sin(dh[14][1] + q[11]), 0, dh[14][2],
             std::sin(dh[14][1] + q[11])*std::cos(dh[14][3]), std::cos(dh[14][3])*std::cos(dh[14][1] + q[11]), -std::sin(dh[14][3]), -dh[14][0]*std::sin(dh[14][3]),
             std::sin(dh[14][3])*std::sin(dh[14][1] + q[11]), std::sin(dh[14][3])*std::cos(dh[14][1] + q[11]), std::cos(dh[14][3]), dh[14][0]*std::cos(dh[14][3]),
             0, 0, 0, 1;
    f(23) << std::cos(dh[15][1] + q[11]), -std::sin(dh[15][1] + q[11]), 0, dh[15][2],
             std::sin(dh[15][1] + q[11])*std::cos(dh[15][3]), std::cos(dh[15][3])*std::cos(dh[15][1] + q[11]), -std::sin(dh[15][3]), -dh[15][0]*std::sin(dh[15][3]),
             std::sin(dh[15][3])*std::sin(dh[15][1] + q[11]), std::sin(dh[15][3])*std::cos(dh[15][1] + q[11]), std::cos(dh[15][3]), dh[15][0]*std::cos(dh[15][3]),
             0, 0, 0, 1;
    f(24) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void fill_frames_dh5(FRAMES& f, JOINTS& q, DH5& dh){
    f(0) << 1., 0, 0, 0,
            0, 1., 0, 0,
            0, 0, 1., 0,
            0, 0, 0, 1.;
    f(1) << -0.140413880, 0.6964277, 0.703755920, -0.0087530,
            -0.168684120, -0.717232940, 0.676108410, -0.04568244,
            0.975617550, -0.02377744, 0.218185760, 0.124312450,
            0, 0, 0, 1.;
    f(2) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) + std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), std::sin(dh[0][4])*std::cos(dh[0][3]), dh[0][2]*std::cos(dh[0][4]) + dh[0][0]*std::sin(dh[0][4])*std::cos(dh[0][3]),
            std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3]), -dh[0][0]*std::sin(dh[0][3]),
            std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) - std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), std::cos(dh[0][3])*std::cos(dh[0][4]), -dh[0][2]*std::sin(dh[0][4]) + dh[0][0]*std::cos(dh[0][3])*std::cos(dh[0][4]),
            0, 0, 0, 1;
    f(3) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) + std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), std::sin(dh[1][4])*std::cos(dh[1][3]), dh[1][2]*std::cos(dh[1][4]) + dh[1][0]*std::sin(dh[1][4])*std::cos(dh[1][3]),
            std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3]), -dh[1][0]*std::sin(dh[1][3]),
            std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) - std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), std::cos(dh[1][3])*std::cos(dh[1][4]), -dh[1][2]*std::sin(dh[1][4]) + dh[1][0]*std::cos(dh[1][3])*std::cos(dh[1][4]),
            0, 0, 0, 1;
    f(4) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) + std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), std::sin(dh[2][4])*std::cos(dh[2][3]), dh[2][2]*std::cos(dh[2][4]) + dh[2][0]*std::sin(dh[2][4])*std::cos(dh[2][3]),
            std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3]), -dh[2][0]*std::sin(dh[2][3]),
            std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) - std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), std::cos(dh[2][3])*std::cos(dh[2][4]), -dh[2][2]*std::sin(dh[2][4]) + dh[2][0]*std::cos(dh[2][3])*std::cos(dh[2][4]),
            0, 0, 0, 1;
    f(5) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]) + std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]), std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]) - std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]), std::sin(dh[3][4])*std::cos(dh[3][3]), dh[3][2]*std::cos(dh[3][4]) + dh[3][0]*std::sin(dh[3][4])*std::cos(dh[3][3]),
            std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3]), -dh[3][0]*std::sin(dh[3][3]),
            std::sin(dh[3][3])*std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]) - std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]), std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]), std::cos(dh[3][3])*std::cos(dh[3][4]), -dh[3][2]*std::sin(dh[3][4]) + dh[3][0]*std::cos(dh[3][3])*std::cos(dh[3][4]),
            0, 0, 0, 1;
    f(6) << -1., 0, 0, 0,
            0, 0, 1., 0.0290,
            0, 1., 0, 0,
            0, 0, 0, 1.;
    f(7) << -0.181930170, 0.02798179, 0.982913240, -0.05103251,
            -0.197346280, -0.980295940, -0.0086201, 0.0029209,
            0.963304650, -0.195542530, 0.183867510, 0.1292965,
            0, 0, 0, 1.;
    f(8) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]) + std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]), std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]) - std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]), std::sin(dh[4][4])*std::cos(dh[4][3]), dh[4][2]*std::cos(dh[4][4]) + dh[4][0]*std::sin(dh[4][4])*std::cos(dh[4][3]),
            std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3]), -dh[4][0]*std::sin(dh[4][3]),
            std::sin(dh[4][3])*std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]) - std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]), std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]), std::cos(dh[4][3])*std::cos(dh[4][4]), -dh[4][2]*std::sin(dh[4][4]) + dh[4][0]*std::cos(dh[4][3])*std::cos(dh[4][4]),
            0, 0, 0, 1;
    f(9) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]) + std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]), std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]) - std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]), std::sin(dh[5][4])*std::cos(dh[5][3]), dh[5][2]*std::cos(dh[5][4]) + dh[5][0]*std::sin(dh[5][4])*std::cos(dh[5][3]),
            std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3]), -dh[5][0]*std::sin(dh[5][3]),
            std::sin(dh[5][3])*std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]) - std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]), std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]), std::cos(dh[5][3])*std::cos(dh[5][4]), -dh[5][2]*std::sin(dh[5][4]) + dh[5][0]*std::cos(dh[5][3])*std::cos(dh[5][4]),
            0, 0, 0, 1;
    f(10) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]) + std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]), std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]) - std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]), std::sin(dh[6][4])*std::cos(dh[6][3]), dh[6][2]*std::cos(dh[6][4]) + dh[6][0]*std::sin(dh[6][4])*std::cos(dh[6][3]),
             std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3]), -dh[6][0]*std::sin(dh[6][3]),
             std::sin(dh[6][3])*std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]) - std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]), std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]), std::cos(dh[6][3])*std::cos(dh[6][4]), -dh[6][2]*std::sin(dh[6][4]) + dh[6][0]*std::cos(dh[6][3])*std::cos(dh[6][4]),
             0, 0, 0, 1;
    f(11) << std::sin(dh[7][3])*std::sin(dh[7][4])*std::sin(dh[7][1] + q[5]) + std::cos(dh[7][4])*std::cos(dh[7][1] + q[5]), std::sin(dh[7][3])*std::sin(dh[7][4])*std::cos(dh[7][1] + q[5]) - std::sin(dh[7][1] + q[5])*std::cos(dh[7][4]), std::sin(dh[7][4])*std::cos(dh[7][3]), dh[7][2]*std::cos(dh[7][4]) + dh[7][0]*std::sin(dh[7][4])*std::cos(dh[7][3]),
             std::sin(dh[7][1] + q[5])*std::cos(dh[7][3]), std::cos(dh[7][3])*std::cos(dh[7][1] + q[5]), -std::sin(dh[7][3]), -dh[7][0]*std::sin(dh[7][3]),
             std::sin(dh[7][3])*std::sin(dh[7][1] + q[5])*std::cos(dh[7][4]) - std::sin(dh[7][4])*std::cos(dh[7][1] + q[5]), std::sin(dh[7][3])*std::cos(dh[7][4])*std::cos(dh[7][1] + q[5]) + std::sin(dh[7][4])*std::sin(dh[7][1] + q[5]), std::cos(dh[7][3])*std::cos(dh[7][4]), -dh[7][2]*std::sin(dh[7][4]) + dh[7][0]*std::cos(dh[7][3])*std::cos(dh[7][4]),
             0, 0, 0, 1;
    f(12) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(13) << -0.210028860, 0.05765612, 0.975993670, -0.04598232,
             -0.09710584, -0.994553860, 0.03785587, 0.02769158,
             0.9728609, -0.08682386, 0.214483760, 0.138843570,
             0, 0, 0, 1.;
    f(14) << std::sin(dh[8][3])*std::sin(dh[8][4])*std::sin(dh[8][1] + q[6]) + std::cos(dh[8][4])*std::cos(dh[8][1] + q[6]), std::sin(dh[8][3])*std::sin(dh[8][4])*std::cos(dh[8][1] + q[6]) - std::sin(dh[8][1] + q[6])*std::cos(dh[8][4]), std::sin(dh[8][4])*std::cos(dh[8][3]), dh[8][2]*std::cos(dh[8][4]) + dh[8][0]*std::sin(dh[8][4])*std::cos(dh[8][3]),
             std::sin(dh[8][1] + q[6])*std::cos(dh[8][3]), std::cos(dh[8][3])*std::cos(dh[8][1] + q[6]), -std::sin(dh[8][3]), -dh[8][0]*std::sin(dh[8][3]),
             std::sin(dh[8][3])*std::sin(dh[8][1] + q[6])*std::cos(dh[8][4]) - std::sin(dh[8][4])*std::cos(dh[8][1] + q[6]), std::sin(dh[8][3])*std::cos(dh[8][4])*std::cos(dh[8][1] + q[6]) + std::sin(dh[8][4])*std::sin(dh[8][1] + q[6]), std::cos(dh[8][3])*std::cos(dh[8][4]), -dh[8][2]*std::sin(dh[8][4]) + dh[8][0]*std::cos(dh[8][3])*std::cos(dh[8][4]),
             0, 0, 0, 1;
    f(15) << std::sin(dh[9][3])*std::sin(dh[9][4])*std::sin(dh[9][1] + q[7]) + std::cos(dh[9][4])*std::cos(dh[9][1] + q[7]), std::sin(dh[9][3])*std::sin(dh[9][4])*std::cos(dh[9][1] + q[7]) - std::sin(dh[9][1] + q[7])*std::cos(dh[9][4]), std::sin(dh[9][4])*std::cos(dh[9][3]), dh[9][2]*std::cos(dh[9][4]) + dh[9][0]*std::sin(dh[9][4])*std::cos(dh[9][3]),
             std::sin(dh[9][1] + q[7])*std::cos(dh[9][3]), std::cos(dh[9][3])*std::cos(dh[9][1] + q[7]), -std::sin(dh[9][3]), -dh[9][0]*std::sin(dh[9][3]),
             std::sin(dh[9][3])*std::sin(dh[9][1] + q[7])*std::cos(dh[9][4]) - std::sin(dh[9][4])*std::cos(dh[9][1] + q[7]), std::sin(dh[9][3])*std::cos(dh[9][4])*std::cos(dh[9][1] + q[7]) + std::sin(dh[9][4])*std::sin(dh[9][1] + q[7]), std::cos(dh[9][3])*std::cos(dh[9][4]), -dh[9][2]*std::sin(dh[9][4]) + dh[9][0]*std::cos(dh[9][3])*std::cos(dh[9][4]),
             0, 0, 0, 1;
    f(16) << std::sin(dh[10][3])*std::sin(dh[10][4])*std::sin(dh[10][1] + q[8]) + std::cos(dh[10][4])*std::cos(dh[10][1] + q[8]), std::sin(dh[10][3])*std::sin(dh[10][4])*std::cos(dh[10][1] + q[8]) - std::sin(dh[10][1] + q[8])*std::cos(dh[10][4]), std::sin(dh[10][4])*std::cos(dh[10][3]), dh[10][2]*std::cos(dh[10][4]) + dh[10][0]*std::sin(dh[10][4])*std::cos(dh[10][3]),
             std::sin(dh[10][1] + q[8])*std::cos(dh[10][3]), std::cos(dh[10][3])*std::cos(dh[10][1] + q[8]), -std::sin(dh[10][3]), -dh[10][0]*std::sin(dh[10][3]),
             std::sin(dh[10][3])*std::sin(dh[10][1] + q[8])*std::cos(dh[10][4]) - std::sin(dh[10][4])*std::cos(dh[10][1] + q[8]), std::sin(dh[10][3])*std::cos(dh[10][4])*std::cos(dh[10][1] + q[8]) + std::sin(dh[10][4])*std::sin(dh[10][1] + q[8]), std::cos(dh[10][3])*std::cos(dh[10][4]), -dh[10][2]*std::sin(dh[10][4]) + dh[10][0]*std::cos(dh[10][3])*std::cos(dh[10][4]),
             0, 0, 0, 1;
    f(17) << std::sin(dh[11][3])*std::sin(dh[11][4])*std::sin(dh[11][1] + q[8]) + std::cos(dh[11][4])*std::cos(dh[11][1] + q[8]), std::sin(dh[11][3])*std::sin(dh[11][4])*std::cos(dh[11][1] + q[8]) - std::sin(dh[11][1] + q[8])*std::cos(dh[11][4]), std::sin(dh[11][4])*std::cos(dh[11][3]), dh[11][2]*std::cos(dh[11][4]) + dh[11][0]*std::sin(dh[11][4])*std::cos(dh[11][3]),
             std::sin(dh[11][1] + q[8])*std::cos(dh[11][3]), std::cos(dh[11][3])*std::cos(dh[11][1] + q[8]), -std::sin(dh[11][3]), -dh[11][0]*std::sin(dh[11][3]),
             std::sin(dh[11][3])*std::sin(dh[11][1] + q[8])*std::cos(dh[11][4]) - std::sin(dh[11][4])*std::cos(dh[11][1] + q[8]), std::sin(dh[11][3])*std::cos(dh[11][4])*std::cos(dh[11][1] + q[8]) + std::sin(dh[11][4])*std::sin(dh[11][1] + q[8]), std::cos(dh[11][3])*std::cos(dh[11][4]), -dh[11][2]*std::sin(dh[11][4]) + dh[11][0]*std::cos(dh[11][3])*std::cos(dh[11][4]),
             0, 0, 0, 1;
    f(18) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    f(19) << 0.430352050, 0.122160610, -0.8943567, 0.05180671,
             -0.337389210, 0.9407565, -0.03384864, 0.03071566,
             0.8372369, 0.316313130, 0.446072160, 0.102831510,
             0, 0, 0, 1.;
    f(20) << std::sin(dh[12][3])*std::sin(dh[12][4])*std::sin(dh[12][1] + q[9]) + std::cos(dh[12][4])*std::cos(dh[12][1] + q[9]), std::sin(dh[12][3])*std::sin(dh[12][4])*std::cos(dh[12][1] + q[9]) - std::sin(dh[12][1] + q[9])*std::cos(dh[12][4]), std::sin(dh[12][4])*std::cos(dh[12][3]), dh[12][2]*std::cos(dh[12][4]) + dh[12][0]*std::sin(dh[12][4])*std::cos(dh[12][3]),
             std::sin(dh[12][1] + q[9])*std::cos(dh[12][3]), std::cos(dh[12][3])*std::cos(dh[12][1] + q[9]), -std::sin(dh[12][3]), -dh[12][0]*std::sin(dh[12][3]),
             std::sin(dh[12][3])*std::sin(dh[12][1] + q[9])*std::cos(dh[12][4]) - std::sin(dh[12][4])*std::cos(dh[12][1] + q[9]), std::sin(dh[12][3])*std::cos(dh[12][4])*std::cos(dh[12][1] + q[9]) + std::sin(dh[12][4])*std::sin(dh[12][1] + q[9]), std::cos(dh[12][3])*std::cos(dh[12][4]), -dh[12][2]*std::sin(dh[12][4]) + dh[12][0]*std::cos(dh[12][3])*std::cos(dh[12][4]),
             0, 0, 0, 1;
    f(21) << std::sin(dh[13][3])*std::sin(dh[13][4])*std::sin(dh[13][1] + q[10]) + std::cos(dh[13][4])*std::cos(dh[13][1] + q[10]), std::sin(dh[13][3])*std::sin(dh[13][4])*std::cos(dh[13][1] + q[10]) - std::sin(dh[13][1] + q[10])*std::cos(dh[13][4]), std::sin(dh[13][4])*std::cos(dh[13][3]), dh[13][2]*std::cos(dh[13][4]) + dh[13][0]*std::sin(dh[13][4])*std::cos(dh[13][3]),
             std::sin(dh[13][1] + q[10])*std::cos(dh[13][3]), std::cos(dh[13][3])*std::cos(dh[13][1] + q[10]), -std::sin(dh[13][3]), -dh[13][0]*std::sin(dh[13][3]),
             std::sin(dh[13][3])*std::sin(dh[13][1] + q[10])*std::cos(dh[13][4]) - std::sin(dh[13][4])*std::cos(dh[13][1] + q[10]), std::sin(dh[13][3])*std::cos(dh[13][4])*std::cos(dh[13][1] + q[10]) + std::sin(dh[13][4])*std::sin(dh[13][1] + q[10]), std::cos(dh[13][3])*std::cos(dh[13][4]), -dh[13][2]*std::sin(dh[13][4]) + dh[13][0]*std::cos(dh[13][3])*std::cos(dh[13][4]),
             0, 0, 0, 1;
    f(22) << std::sin(dh[14][3])*std::sin(dh[14][4])*std::sin(dh[14][1] + q[11]) + std::cos(dh[14][4])*std::cos(dh[14][1] + q[11]), std::sin(dh[14][3])*std::sin(dh[14][4])*std::cos(dh[14][1] + q[11]) - std::sin(dh[14][1] + q[11])*std::cos(dh[14][4]), std::sin(dh[14][4])*std::cos(dh[14][3]), dh[14][2]*std::cos(dh[14][4]) + dh[14][0]*std::sin(dh[14][4])*std::cos(dh[14][3]),
             std::sin(dh[14][1] + q[11])*std::cos(dh[14][3]), std::cos(dh[14][3])*std::cos(dh[14][1] + q[11]), -std::sin(dh[14][3]), -dh[14][0]*std::sin(dh[14][3]),
             std::sin(dh[14][3])*std::sin(dh[14][1] + q[11])*std::cos(dh[14][4]) - std::sin(dh[14][4])*std::cos(dh[14][1] + q[11]), std::sin(dh[14][3])*std::cos(dh[14][4])*std::cos(dh[14][1] + q[11]) + std::sin(dh[14][4])*std::sin(dh[14][1] + q[11]), std::cos(dh[14][3])*std::cos(dh[14][4]), -dh[14][2]*std::sin(dh[14][4]) + dh[14][0]*std::cos(dh[14][3])*std::cos(dh[14][4]),
             0, 0, 0, 1;
    f(23) << std::sin(dh[15][3])*std::sin(dh[15][4])*std::sin(dh[15][1] + q[11]) + std::cos(dh[15][4])*std::cos(dh[15][1] + q[11]), std::sin(dh[15][3])*std::sin(dh[15][4])*std::cos(dh[15][1] + q[11]) - std::sin(dh[15][1] + q[11])*std::cos(dh[15][4]), std::sin(dh[15][4])*std::cos(dh[15][3]), dh[15][2]*std::cos(dh[15][4]) + dh[15][0]*std::sin(dh[15][4])*std::cos(dh[15][3]),
             std::sin(dh[15][1] + q[11])*std::cos(dh[15][3]), std::cos(dh[15][3])*std::cos(dh[15][1] + q[11]), -std::sin(dh[15][3]), -dh[15][0]*std::sin(dh[15][3]),
             std::sin(dh[15][3])*std::sin(dh[15][1] + q[11])*std::cos(dh[15][4]) - std::sin(dh[15][4])*std::cos(dh[15][1] + q[11]), std::sin(dh[15][3])*std::cos(dh[15][4])*std::cos(dh[15][1] + q[11]) + std::sin(dh[15][4])*std::sin(dh[15][1] + q[11]), std::cos(dh[15][3])*std::cos(dh[15][4]), -dh[15][2]*std::sin(dh[15][4]) + dh[15][0]*std::cos(dh[15][3])*std::cos(dh[15][4]),
             0, 0, 0, 1;
    f(24) << -1., 0, 0, 0,
             0, 0, 1., 0.0290,
             0, 1., 0, 0,
             0, 0, 0, 1.;
    
}


inline void _fill_jac(JACS& j, JOINTS& q){
    j(0, 2) << -std::sin(q[0]), -std::cos(q[0]), 0, 0,
               std::cos(q[0]), -std::sin(q[0]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(1, 3) << -std::sin(q[1]), -std::cos(q[1]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[1]), -std::sin(q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(q[2]), -std::cos(q[2]), 0, 0,
               std::cos(q[2]), -std::sin(q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(2, 5) << std::cos(q[2]), -std::sin(q[2]), 0, 0,
               std::cos(q[2] - 0.5*pi), std::cos(q[2]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(3, 8) << -std::sin(q[3]), -std::cos(q[3]), 0, 0,
               std::cos(q[3]), -std::sin(q[3]), 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    j(4, 9) << -std::sin(q[4]), -std::cos(q[4]), 0, 0,
               0, 0, 0, 0,
               std::cos(q[4]), -std::sin(q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 10) << -std::sin(q[5]), -std::cos(q[5]), 0, 0,
                std::cos(q[5]), -std::sin(q[5]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(5, 11) << std::cos(q[5]), -std::sin(q[5]), 0, 0,
                std::cos(q[5] - 0.5*pi), std::cos(q[5]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(6, 14) << -std::sin(q[6]), -std::cos(q[6]), 0, 0,
                std::cos(q[6]), -std::sin(q[6]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(7, 15) << -std::sin(q[7]), -std::cos(q[7]), 0, 0,
                0, 0, 0, 0,
                std::cos(q[7]), -std::sin(q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 16) << -std::sin(q[8]), -std::cos(q[8]), 0, 0,
                std::cos(q[8]), -std::sin(q[8]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(8, 17) << std::cos(q[8]), -std::sin(q[8]), 0, 0,
                std::cos(q[8] - 0.5*pi), std::cos(q[8]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(9, 20) << -std::sin(q[9]), -std::cos(q[9]), 0, 0,
                std::cos(q[9]), -std::sin(q[9]), 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    j(10, 21) << -std::sin(q[10]), -std::cos(q[10]), 0, 0,
                 0, 0, 0, 0,
                 std::cos(q[10]), -std::sin(q[10]), 0, 0,
                 0, 0, 0, 0;
    j(11, 22) << -std::sin(q[11]), -std::cos(q[11]), 0, 0,
                 std::cos(q[11]), -std::sin(q[11]), 0, 0,
                 0, 0, 0, 0,
                 0, 0, 0, 0;
    j(11, 23) << std::cos(q[11]), -std::sin(q[11]), 0, 0,
                 std::cos(q[11] - 0.5*pi), std::cos(q[11]), 0, 0,
                 0, 0, 0, 0,
                 0, 0, 0, 0;
    
}


inline void fill_jacs_dh4(JACS& j, JOINTS& q, DH4& dh){
    j(0, 2) << -std::sin(dh[0][1] + q[0]), -std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 3) << -std::sin(dh[1][1] + q[1]), -std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << -std::sin(dh[2][1] + q[2]), -std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 5) << -std::sin(dh[3][1] + q[2]), -std::cos(dh[3][1] + q[2]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 8) << -std::sin(dh[4][1] + q[3]), -std::cos(dh[4][1] + q[3]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 9) << -std::sin(dh[5][1] + q[4]), -std::cos(dh[5][1] + q[4]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 10) << -std::sin(dh[6][1] + q[5]), -std::cos(dh[6][1] + q[5]), 0, 0,
                std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), 0, 0,
                std::sin(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[5]), 0, 0,
                0, 0, 0, 0;
    j(5, 11) << -std::sin(dh[7][1] + q[5]), -std::cos(dh[7][1] + q[5]), 0, 0,
                std::cos(dh[7][3])*std::cos(dh[7][1] + q[5]), -std::sin(dh[7][1] + q[5])*std::cos(dh[7][3]), 0, 0,
                std::sin(dh[7][3])*std::cos(dh[7][1] + q[5]), -std::sin(dh[7][3])*std::sin(dh[7][1] + q[5]), 0, 0,
                0, 0, 0, 0;
    j(6, 14) << -std::sin(dh[8][1] + q[6]), -std::cos(dh[8][1] + q[6]), 0, 0,
                std::cos(dh[8][3])*std::cos(dh[8][1] + q[6]), -std::sin(dh[8][1] + q[6])*std::cos(dh[8][3]), 0, 0,
                std::sin(dh[8][3])*std::cos(dh[8][1] + q[6]), -std::sin(dh[8][3])*std::sin(dh[8][1] + q[6]), 0, 0,
                0, 0, 0, 0;
    j(7, 15) << -std::sin(dh[9][1] + q[7]), -std::cos(dh[9][1] + q[7]), 0, 0,
                std::cos(dh[9][3])*std::cos(dh[9][1] + q[7]), -std::sin(dh[9][1] + q[7])*std::cos(dh[9][3]), 0, 0,
                std::sin(dh[9][3])*std::cos(dh[9][1] + q[7]), -std::sin(dh[9][3])*std::sin(dh[9][1] + q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 16) << -std::sin(dh[10][1] + q[8]), -std::cos(dh[10][1] + q[8]), 0, 0,
                std::cos(dh[10][3])*std::cos(dh[10][1] + q[8]), -std::sin(dh[10][1] + q[8])*std::cos(dh[10][3]), 0, 0,
                std::sin(dh[10][3])*std::cos(dh[10][1] + q[8]), -std::sin(dh[10][3])*std::sin(dh[10][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(8, 17) << -std::sin(dh[11][1] + q[8]), -std::cos(dh[11][1] + q[8]), 0, 0,
                std::cos(dh[11][3])*std::cos(dh[11][1] + q[8]), -std::sin(dh[11][1] + q[8])*std::cos(dh[11][3]), 0, 0,
                std::sin(dh[11][3])*std::cos(dh[11][1] + q[8]), -std::sin(dh[11][3])*std::sin(dh[11][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(9, 20) << -std::sin(dh[12][1] + q[9]), -std::cos(dh[12][1] + q[9]), 0, 0,
                std::cos(dh[12][3])*std::cos(dh[12][1] + q[9]), -std::sin(dh[12][1] + q[9])*std::cos(dh[12][3]), 0, 0,
                std::sin(dh[12][3])*std::cos(dh[12][1] + q[9]), -std::sin(dh[12][3])*std::sin(dh[12][1] + q[9]), 0, 0,
                0, 0, 0, 0;
    j(10, 21) << -std::sin(dh[13][1] + q[10]), -std::cos(dh[13][1] + q[10]), 0, 0,
                 std::cos(dh[13][3])*std::cos(dh[13][1] + q[10]), -std::sin(dh[13][1] + q[10])*std::cos(dh[13][3]), 0, 0,
                 std::sin(dh[13][3])*std::cos(dh[13][1] + q[10]), -std::sin(dh[13][3])*std::sin(dh[13][1] + q[10]), 0, 0,
                 0, 0, 0, 0;
    j(11, 22) << -std::sin(dh[14][1] + q[11]), -std::cos(dh[14][1] + q[11]), 0, 0,
                 std::cos(dh[14][3])*std::cos(dh[14][1] + q[11]), -std::sin(dh[14][1] + q[11])*std::cos(dh[14][3]), 0, 0,
                 std::sin(dh[14][3])*std::cos(dh[14][1] + q[11]), -std::sin(dh[14][3])*std::sin(dh[14][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    j(11, 23) << -std::sin(dh[15][1] + q[11]), -std::cos(dh[15][1] + q[11]), 0, 0,
                 std::cos(dh[15][3])*std::cos(dh[15][1] + q[11]), -std::sin(dh[15][1] + q[11])*std::cos(dh[15][3]), 0, 0,
                 std::sin(dh[15][3])*std::cos(dh[15][1] + q[11]), -std::sin(dh[15][3])*std::sin(dh[15][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    
}


inline void fill_jacs_dh5(JACS& j, JOINTS& q, DH5& dh){
    j(0, 2) << std::sin(dh[0][3])*std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]) - std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]), -std::sin(dh[0][3])*std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]) - std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               std::cos(dh[0][3])*std::cos(dh[0][1] + q[0]), -std::sin(dh[0][1] + q[0])*std::cos(dh[0][3]), 0, 0,
               std::sin(dh[0][3])*std::cos(dh[0][4])*std::cos(dh[0][1] + q[0]) + std::sin(dh[0][4])*std::sin(dh[0][1] + q[0]), -std::sin(dh[0][3])*std::sin(dh[0][1] + q[0])*std::cos(dh[0][4]) + std::sin(dh[0][4])*std::cos(dh[0][1] + q[0]), 0, 0,
               0, 0, 0, 0;
    j(1, 3) << std::sin(dh[1][3])*std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]) - std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]), -std::sin(dh[1][3])*std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]) - std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               std::cos(dh[1][3])*std::cos(dh[1][1] + q[1]), -std::sin(dh[1][1] + q[1])*std::cos(dh[1][3]), 0, 0,
               std::sin(dh[1][3])*std::cos(dh[1][4])*std::cos(dh[1][1] + q[1]) + std::sin(dh[1][4])*std::sin(dh[1][1] + q[1]), -std::sin(dh[1][3])*std::sin(dh[1][1] + q[1])*std::cos(dh[1][4]) + std::sin(dh[1][4])*std::cos(dh[1][1] + q[1]), 0, 0,
               0, 0, 0, 0;
    j(2, 4) << std::sin(dh[2][3])*std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]) - std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]), -std::sin(dh[2][3])*std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]) - std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               std::cos(dh[2][3])*std::cos(dh[2][1] + q[2]), -std::sin(dh[2][1] + q[2])*std::cos(dh[2][3]), 0, 0,
               std::sin(dh[2][3])*std::cos(dh[2][4])*std::cos(dh[2][1] + q[2]) + std::sin(dh[2][4])*std::sin(dh[2][1] + q[2]), -std::sin(dh[2][3])*std::sin(dh[2][1] + q[2])*std::cos(dh[2][4]) + std::sin(dh[2][4])*std::cos(dh[2][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(2, 5) << std::sin(dh[3][3])*std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]) - std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]), -std::sin(dh[3][3])*std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]) - std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]), 0, 0,
               std::cos(dh[3][3])*std::cos(dh[3][1] + q[2]), -std::sin(dh[3][1] + q[2])*std::cos(dh[3][3]), 0, 0,
               std::sin(dh[3][3])*std::cos(dh[3][4])*std::cos(dh[3][1] + q[2]) + std::sin(dh[3][4])*std::sin(dh[3][1] + q[2]), -std::sin(dh[3][3])*std::sin(dh[3][1] + q[2])*std::cos(dh[3][4]) + std::sin(dh[3][4])*std::cos(dh[3][1] + q[2]), 0, 0,
               0, 0, 0, 0;
    j(3, 8) << std::sin(dh[4][3])*std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]) - std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]), -std::sin(dh[4][3])*std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]) - std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]), 0, 0,
               std::cos(dh[4][3])*std::cos(dh[4][1] + q[3]), -std::sin(dh[4][1] + q[3])*std::cos(dh[4][3]), 0, 0,
               std::sin(dh[4][3])*std::cos(dh[4][4])*std::cos(dh[4][1] + q[3]) + std::sin(dh[4][4])*std::sin(dh[4][1] + q[3]), -std::sin(dh[4][3])*std::sin(dh[4][1] + q[3])*std::cos(dh[4][4]) + std::sin(dh[4][4])*std::cos(dh[4][1] + q[3]), 0, 0,
               0, 0, 0, 0;
    j(4, 9) << std::sin(dh[5][3])*std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]) - std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]), -std::sin(dh[5][3])*std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]) - std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]), 0, 0,
               std::cos(dh[5][3])*std::cos(dh[5][1] + q[4]), -std::sin(dh[5][1] + q[4])*std::cos(dh[5][3]), 0, 0,
               std::sin(dh[5][3])*std::cos(dh[5][4])*std::cos(dh[5][1] + q[4]) + std::sin(dh[5][4])*std::sin(dh[5][1] + q[4]), -std::sin(dh[5][3])*std::sin(dh[5][1] + q[4])*std::cos(dh[5][4]) + std::sin(dh[5][4])*std::cos(dh[5][1] + q[4]), 0, 0,
               0, 0, 0, 0;
    j(5, 10) << std::sin(dh[6][3])*std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]) - std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]), -std::sin(dh[6][3])*std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]) - std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]), 0, 0,
                std::cos(dh[6][3])*std::cos(dh[6][1] + q[5]), -std::sin(dh[6][1] + q[5])*std::cos(dh[6][3]), 0, 0,
                std::sin(dh[6][3])*std::cos(dh[6][4])*std::cos(dh[6][1] + q[5]) + std::sin(dh[6][4])*std::sin(dh[6][1] + q[5]), -std::sin(dh[6][3])*std::sin(dh[6][1] + q[5])*std::cos(dh[6][4]) + std::sin(dh[6][4])*std::cos(dh[6][1] + q[5]), 0, 0,
                0, 0, 0, 0;
    j(5, 11) << std::sin(dh[7][3])*std::sin(dh[7][4])*std::cos(dh[7][1] + q[5]) - std::sin(dh[7][1] + q[5])*std::cos(dh[7][4]), -std::sin(dh[7][3])*std::sin(dh[7][4])*std::sin(dh[7][1] + q[5]) - std::cos(dh[7][4])*std::cos(dh[7][1] + q[5]), 0, 0,
                std::cos(dh[7][3])*std::cos(dh[7][1] + q[5]), -std::sin(dh[7][1] + q[5])*std::cos(dh[7][3]), 0, 0,
                std::sin(dh[7][3])*std::cos(dh[7][4])*std::cos(dh[7][1] + q[5]) + std::sin(dh[7][4])*std::sin(dh[7][1] + q[5]), -std::sin(dh[7][3])*std::sin(dh[7][1] + q[5])*std::cos(dh[7][4]) + std::sin(dh[7][4])*std::cos(dh[7][1] + q[5]), 0, 0,
                0, 0, 0, 0;
    j(6, 14) << std::sin(dh[8][3])*std::sin(dh[8][4])*std::cos(dh[8][1] + q[6]) - std::sin(dh[8][1] + q[6])*std::cos(dh[8][4]), -std::sin(dh[8][3])*std::sin(dh[8][4])*std::sin(dh[8][1] + q[6]) - std::cos(dh[8][4])*std::cos(dh[8][1] + q[6]), 0, 0,
                std::cos(dh[8][3])*std::cos(dh[8][1] + q[6]), -std::sin(dh[8][1] + q[6])*std::cos(dh[8][3]), 0, 0,
                std::sin(dh[8][3])*std::cos(dh[8][4])*std::cos(dh[8][1] + q[6]) + std::sin(dh[8][4])*std::sin(dh[8][1] + q[6]), -std::sin(dh[8][3])*std::sin(dh[8][1] + q[6])*std::cos(dh[8][4]) + std::sin(dh[8][4])*std::cos(dh[8][1] + q[6]), 0, 0,
                0, 0, 0, 0;
    j(7, 15) << std::sin(dh[9][3])*std::sin(dh[9][4])*std::cos(dh[9][1] + q[7]) - std::sin(dh[9][1] + q[7])*std::cos(dh[9][4]), -std::sin(dh[9][3])*std::sin(dh[9][4])*std::sin(dh[9][1] + q[7]) - std::cos(dh[9][4])*std::cos(dh[9][1] + q[7]), 0, 0,
                std::cos(dh[9][3])*std::cos(dh[9][1] + q[7]), -std::sin(dh[9][1] + q[7])*std::cos(dh[9][3]), 0, 0,
                std::sin(dh[9][3])*std::cos(dh[9][4])*std::cos(dh[9][1] + q[7]) + std::sin(dh[9][4])*std::sin(dh[9][1] + q[7]), -std::sin(dh[9][3])*std::sin(dh[9][1] + q[7])*std::cos(dh[9][4]) + std::sin(dh[9][4])*std::cos(dh[9][1] + q[7]), 0, 0,
                0, 0, 0, 0;
    j(8, 16) << std::sin(dh[10][3])*std::sin(dh[10][4])*std::cos(dh[10][1] + q[8]) - std::sin(dh[10][1] + q[8])*std::cos(dh[10][4]), -std::sin(dh[10][3])*std::sin(dh[10][4])*std::sin(dh[10][1] + q[8]) - std::cos(dh[10][4])*std::cos(dh[10][1] + q[8]), 0, 0,
                std::cos(dh[10][3])*std::cos(dh[10][1] + q[8]), -std::sin(dh[10][1] + q[8])*std::cos(dh[10][3]), 0, 0,
                std::sin(dh[10][3])*std::cos(dh[10][4])*std::cos(dh[10][1] + q[8]) + std::sin(dh[10][4])*std::sin(dh[10][1] + q[8]), -std::sin(dh[10][3])*std::sin(dh[10][1] + q[8])*std::cos(dh[10][4]) + std::sin(dh[10][4])*std::cos(dh[10][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(8, 17) << std::sin(dh[11][3])*std::sin(dh[11][4])*std::cos(dh[11][1] + q[8]) - std::sin(dh[11][1] + q[8])*std::cos(dh[11][4]), -std::sin(dh[11][3])*std::sin(dh[11][4])*std::sin(dh[11][1] + q[8]) - std::cos(dh[11][4])*std::cos(dh[11][1] + q[8]), 0, 0,
                std::cos(dh[11][3])*std::cos(dh[11][1] + q[8]), -std::sin(dh[11][1] + q[8])*std::cos(dh[11][3]), 0, 0,
                std::sin(dh[11][3])*std::cos(dh[11][4])*std::cos(dh[11][1] + q[8]) + std::sin(dh[11][4])*std::sin(dh[11][1] + q[8]), -std::sin(dh[11][3])*std::sin(dh[11][1] + q[8])*std::cos(dh[11][4]) + std::sin(dh[11][4])*std::cos(dh[11][1] + q[8]), 0, 0,
                0, 0, 0, 0;
    j(9, 20) << std::sin(dh[12][3])*std::sin(dh[12][4])*std::cos(dh[12][1] + q[9]) - std::sin(dh[12][1] + q[9])*std::cos(dh[12][4]), -std::sin(dh[12][3])*std::sin(dh[12][4])*std::sin(dh[12][1] + q[9]) - std::cos(dh[12][4])*std::cos(dh[12][1] + q[9]), 0, 0,
                std::cos(dh[12][3])*std::cos(dh[12][1] + q[9]), -std::sin(dh[12][1] + q[9])*std::cos(dh[12][3]), 0, 0,
                std::sin(dh[12][3])*std::cos(dh[12][4])*std::cos(dh[12][1] + q[9]) + std::sin(dh[12][4])*std::sin(dh[12][1] + q[9]), -std::sin(dh[12][3])*std::sin(dh[12][1] + q[9])*std::cos(dh[12][4]) + std::sin(dh[12][4])*std::cos(dh[12][1] + q[9]), 0, 0,
                0, 0, 0, 0;
    j(10, 21) << std::sin(dh[13][3])*std::sin(dh[13][4])*std::cos(dh[13][1] + q[10]) - std::sin(dh[13][1] + q[10])*std::cos(dh[13][4]), -std::sin(dh[13][3])*std::sin(dh[13][4])*std::sin(dh[13][1] + q[10]) - std::cos(dh[13][4])*std::cos(dh[13][1] + q[10]), 0, 0,
                 std::cos(dh[13][3])*std::cos(dh[13][1] + q[10]), -std::sin(dh[13][1] + q[10])*std::cos(dh[13][3]), 0, 0,
                 std::sin(dh[13][3])*std::cos(dh[13][4])*std::cos(dh[13][1] + q[10]) + std::sin(dh[13][4])*std::sin(dh[13][1] + q[10]), -std::sin(dh[13][3])*std::sin(dh[13][1] + q[10])*std::cos(dh[13][4]) + std::sin(dh[13][4])*std::cos(dh[13][1] + q[10]), 0, 0,
                 0, 0, 0, 0;
    j(11, 22) << std::sin(dh[14][3])*std::sin(dh[14][4])*std::cos(dh[14][1] + q[11]) - std::sin(dh[14][1] + q[11])*std::cos(dh[14][4]), -std::sin(dh[14][3])*std::sin(dh[14][4])*std::sin(dh[14][1] + q[11]) - std::cos(dh[14][4])*std::cos(dh[14][1] + q[11]), 0, 0,
                 std::cos(dh[14][3])*std::cos(dh[14][1] + q[11]), -std::sin(dh[14][1] + q[11])*std::cos(dh[14][3]), 0, 0,
                 std::sin(dh[14][3])*std::cos(dh[14][4])*std::cos(dh[14][1] + q[11]) + std::sin(dh[14][4])*std::sin(dh[14][1] + q[11]), -std::sin(dh[14][3])*std::sin(dh[14][1] + q[11])*std::cos(dh[14][4]) + std::sin(dh[14][4])*std::cos(dh[14][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    j(11, 23) << std::sin(dh[15][3])*std::sin(dh[15][4])*std::cos(dh[15][1] + q[11]) - std::sin(dh[15][1] + q[11])*std::cos(dh[15][4]), -std::sin(dh[15][3])*std::sin(dh[15][4])*std::sin(dh[15][1] + q[11]) - std::cos(dh[15][4])*std::cos(dh[15][1] + q[11]), 0, 0,
                 std::cos(dh[15][3])*std::cos(dh[15][1] + q[11]), -std::sin(dh[15][1] + q[11])*std::cos(dh[15][3]), 0, 0,
                 std::sin(dh[15][3])*std::cos(dh[15][4])*std::cos(dh[15][1] + q[11]) + std::sin(dh[15][4])*std::sin(dh[15][1] + q[11]), -std::sin(dh[15][3])*std::sin(dh[15][1] + q[11])*std::cos(dh[15][4]) + std::sin(dh[15][4])*std::cos(dh[15][1] + q[11]), 0, 0,
                 0, 0, 0, 0;
    
}


void combine_frames(FRAMES& f){
    f(1) = f(0) * f(1);
    f(2) = f(1) * f(2);
    f(3) = f(2) * f(3);
    f(4) = f(3) * f(4);
    f(5) = f(4) * f(5);
    f(6) = f(5) * f(6);
    f(7) = f(0) * f(7);
    f(8) = f(7) * f(8);
    f(9) = f(8) * f(9);
    f(10) = f(9) * f(10);
    f(11) = f(10) * f(11);
    f(12) = f(11) * f(12);
    f(13) = f(0) * f(13);
    f(14) = f(13) * f(14);
    f(15) = f(14) * f(15);
    f(16) = f(15) * f(16);
    f(17) = f(16) * f(17);
    f(18) = f(17) * f(18);
    f(19) = f(0) * f(19);
    f(20) = f(19) * f(20);
    f(21) = f(20) * f(21);
    f(22) = f(21) * f(22);
    f(23) = f(22) * f(23);
    f(24) = f(23) * f(24);
    
}


void combine_jacs(JACS& j, DICT& d){
    // 0
    j(0, 1) = d(0, 0) * j(0, 1);
    j(0, 2) = j(0, 1) * d(2, 2);
    j(0, 3) = j(0, 1) * d(2, 3);
    j(0, 4) = j(0, 1) * d(2, 4);
    j(0, 5) = j(0, 1) * d(2, 5);
    j(0, 6) = j(0, 1) * d(2, 6);
    // 1
    j(1, 2) = d(0, 1) * j(1, 2);
    j(1, 3) = j(1, 2) * d(3, 3);
    j(1, 4) = j(1, 2) * d(3, 4);
    j(1, 5) = j(1, 2) * d(3, 5);
    j(1, 6) = j(1, 2) * d(3, 6);
    // 2
    j(2, 3) = d(0, 2) * j(2, 3);
    j(2, 4) = j(2, 3) * d(4, 4) + d(0, 3) * j(2, 4);
    j(2, 5) = j(2, 4) * d(5, 5);
    j(2, 6) = j(2, 4) * d(5, 6);
    // 3
    j(3, 7) = d(0, 0) * j(3, 7);
    j(3, 8) = j(3, 7) * d(8, 8);
    j(3, 9) = j(3, 7) * d(8, 9);
    j(3, 10) = j(3, 7) * d(8, 10);
    j(3, 11) = j(3, 7) * d(8, 11);
    j(3, 12) = j(3, 7) * d(8, 12);
    // 4
    j(4, 8) = d(0, 7) * j(4, 8);
    j(4, 9) = j(4, 8) * d(9, 9);
    j(4, 10) = j(4, 8) * d(9, 10);
    j(4, 11) = j(4, 8) * d(9, 11);
    j(4, 12) = j(4, 8) * d(9, 12);
    // 5
    j(5, 9) = d(0, 8) * j(5, 9);
    j(5, 10) = j(5, 9) * d(10, 10) + d(0, 9) * j(5, 10);
    j(5, 11) = j(5, 10) * d(11, 11);
    j(5, 12) = j(5, 10) * d(11, 12);
    // 6
    j(6, 13) = d(0, 0) * j(6, 13);
    j(6, 14) = j(6, 13) * d(14, 14);
    j(6, 15) = j(6, 13) * d(14, 15);
    j(6, 16) = j(6, 13) * d(14, 16);
    j(6, 17) = j(6, 13) * d(14, 17);
    j(6, 18) = j(6, 13) * d(14, 18);
    // 7
    j(7, 14) = d(0, 13) * j(7, 14);
    j(7, 15) = j(7, 14) * d(15, 15);
    j(7, 16) = j(7, 14) * d(15, 16);
    j(7, 17) = j(7, 14) * d(15, 17);
    j(7, 18) = j(7, 14) * d(15, 18);
    // 8
    j(8, 15) = d(0, 14) * j(8, 15);
    j(8, 16) = j(8, 15) * d(16, 16) + d(0, 15) * j(8, 16);
    j(8, 17) = j(8, 16) * d(17, 17);
    j(8, 18) = j(8, 16) * d(17, 18);
    // 9
    j(9, 19) = d(0, 0) * j(9, 19);
    j(9, 20) = j(9, 19) * d(20, 20);
    j(9, 21) = j(9, 19) * d(20, 21);
    j(9, 22) = j(9, 19) * d(20, 22);
    j(9, 23) = j(9, 19) * d(20, 23);
    j(9, 24) = j(9, 19) * d(20, 24);
    // 10
    j(10, 20) = d(0, 19) * j(10, 20);
    j(10, 21) = j(10, 20) * d(21, 21);
    j(10, 22) = j(10, 20) * d(21, 22);
    j(10, 23) = j(10, 20) * d(21, 23);
    j(10, 24) = j(10, 20) * d(21, 24);
    // 11
    j(11, 21) = d(0, 20) * j(11, 21);
    j(11, 22) = j(11, 21) * d(22, 22) + d(0, 21) * j(11, 22);
    j(11, 23) = j(11, 22) * d(23, 23);
    j(11, 24) = j(11, 22) * d(23, 24);
    
}


void combine_dict(FRAMES& f, DICT& d){
    // 24
    d(24, 24) = f(24);
    // 23
    d(23, 23) = f(23);
    d(23, 24) = f(23) * d(24, 24);
    // 22
    d(22, 22) = f(22);
    d(22, 23) = f(22) * d(23, 23);
    d(22, 24) = f(22) * d(23, 24);
    // 21
    d(21, 21) = f(21);
    d(21, 22) = f(21) * d(22, 22);
    d(21, 23) = f(21) * d(22, 23);
    d(21, 24) = f(21) * d(22, 24);
    // 20
    d(20, 20) = f(20);
    d(20, 21) = f(20) * d(21, 21);
    d(20, 22) = f(20) * d(21, 22);
    d(20, 23) = f(20) * d(21, 23);
    d(20, 24) = f(20) * d(21, 24);
    // 19
    d(19, 19) = f(19);
    d(19, 20) = f(19) * d(20, 20);
    d(19, 21) = f(19) * d(20, 21);
    d(19, 22) = f(19) * d(20, 22);
    d(19, 23) = f(19) * d(20, 23);
    d(19, 24) = f(19) * d(20, 24);
    // 18
    d(18, 18) = f(18);
    // 17
    d(17, 17) = f(17);
    d(17, 18) = f(17) * d(18, 18);
    // 16
    d(16, 16) = f(16);
    d(16, 17) = f(16) * d(17, 17);
    d(16, 18) = f(16) * d(17, 18);
    // 15
    d(15, 15) = f(15);
    d(15, 16) = f(15) * d(16, 16);
    d(15, 17) = f(15) * d(16, 17);
    d(15, 18) = f(15) * d(16, 18);
    // 14
    d(14, 14) = f(14);
    d(14, 15) = f(14) * d(15, 15);
    d(14, 16) = f(14) * d(15, 16);
    d(14, 17) = f(14) * d(15, 17);
    d(14, 18) = f(14) * d(15, 18);
    // 13
    d(13, 13) = f(13);
    d(13, 14) = f(13) * d(14, 14);
    d(13, 15) = f(13) * d(14, 15);
    d(13, 16) = f(13) * d(14, 16);
    d(13, 17) = f(13) * d(14, 17);
    d(13, 18) = f(13) * d(14, 18);
    // 12
    d(12, 12) = f(12);
    // 11
    d(11, 11) = f(11);
    d(11, 12) = f(11) * d(12, 12);
    // 10
    d(10, 10) = f(10);
    d(10, 11) = f(10) * d(11, 11);
    d(10, 12) = f(10) * d(11, 12);
    // 9
    d(9, 9) = f(9);
    d(9, 10) = f(9) * d(10, 10);
    d(9, 11) = f(9) * d(10, 11);
    d(9, 12) = f(9) * d(10, 12);
    // 8
    d(8, 8) = f(8);
    d(8, 9) = f(8) * d(9, 9);
    d(8, 10) = f(8) * d(9, 10);
    d(8, 11) = f(8) * d(9, 11);
    d(8, 12) = f(8) * d(9, 12);
    // 7
    d(7, 7) = f(7);
    d(7, 8) = f(7) * d(8, 8);
    d(7, 9) = f(7) * d(8, 9);
    d(7, 10) = f(7) * d(8, 10);
    d(7, 11) = f(7) * d(8, 11);
    d(7, 12) = f(7) * d(8, 12);
    // 6
    d(6, 6) = f(6);
    // 5
    d(5, 5) = f(5);
    d(5, 6) = f(5) * d(6, 6);
    // 4
    d(4, 4) = f(4);
    d(4, 5) = f(4) * d(5, 5);
    d(4, 6) = f(4) * d(5, 6);
    // 3
    d(3, 3) = f(3);
    d(3, 4) = f(3) * d(4, 4);
    d(3, 5) = f(3) * d(4, 5);
    d(3, 6) = f(3) * d(4, 6);
    // 2
    d(2, 2) = f(2);
    d(2, 3) = f(2) * d(3, 3);
    d(2, 4) = f(2) * d(3, 4);
    d(2, 5) = f(2) * d(3, 5);
    d(2, 6) = f(2) * d(3, 6);
    // 1
    d(1, 1) = f(1);
    d(1, 2) = f(1) * d(2, 2);
    d(1, 3) = f(1) * d(2, 3);
    d(1, 4) = f(1) * d(2, 4);
    d(1, 5) = f(1) * d(2, 5);
    d(1, 6) = f(1) * d(2, 6);
    // 0
    d(0, 0) = f(0);
    d(0, 1) = f(0) * d(1, 1);
    d(0, 2) = f(0) * d(1, 2);
    d(0, 3) = f(0) * d(1, 3);
    d(0, 4) = f(0) * d(1, 4);
    d(0, 5) = f(0) * d(1, 5);
    d(0, 6) = f(0) * d(1, 6);
    
}

    