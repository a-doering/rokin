#ifndef JUSTIN19_H
#define JUSTIN19_H

#include <Eigen/Dense>

const int N_JOINTS = 19;
const int N_FRAMES = 27;
const int N_DH = 20;
const double pi = 3.141592653589793;

typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> MatrixRow;
typedef Eigen::Map<Eigen::Array< MatrixRow, N_FRAMES, 1, Eigen::ColMajor> > FRAMES;
typedef Eigen::Map<Eigen::Array< MatrixRow,  N_JOINTS, N_FRAMES, Eigen::RowMajor> > JACS;
typedef Eigen::Array< MatrixRow, N_FRAMES, N_FRAMES, Eigen::RowMajor> DICT;
typedef double JOINTS[N_JOINTS];
typedef double DH4[N_DH][4];
typedef double DH5[N_DH][5];

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

#endif // JUSTIN19_H

    