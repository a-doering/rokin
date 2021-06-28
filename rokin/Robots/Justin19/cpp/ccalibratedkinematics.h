#ifndef CCALIBRATEDKINEMATICS_H
#define CCALIBRATEDKINEMATICS_H

#include "Justin19.h"

const int OMP_OO_MAX_NUM_OF_LINKS = 32;
typedef double FRAMES32[OMP_OO_MAX_NUM_OF_LINKS][4][4];
void crkw_robot_kinematics(JOINTS& q, FRAMES32& frames32);

#endif // CCALIBRATEDKINEMATICS_H
