#include <stdio.h>
#include "ccalibratedkinematics.h"
//#include "Justin19.h"


int main(void)
{
    JOINTS q = {0};
    FRAMES32 frames32;;
    crkw_robot_kinematics(q, frames32);

    int i = 13;
    printf("(Frame 13):\n");
    printf("%.3f %.3f %.3f %.3f\n", frames32[i][0][0], frames32[i][0][1], frames32[i][0][2], frames32[i][0][3]);
    printf("%.3f %.3f %.3f %.3f\n", frames32[i][1][0], frames32[i][1][1], frames32[i][1][2], frames32[i][1][3]);
    printf("%.3f %.3f %.3f %.3f\n", frames32[i][2][0], frames32[i][2][1], frames32[i][2][2], frames32[i][2][3]);
    printf("%.3f %.3f %.3f %.3f\n", frames32[i][3][0], frames32[i][3][1], frames32[i][3][2], frames32[i][3][3]);
    return 0;
}
