#include <cmath>
#include <Eigen/Dense>

#include "ccalibratedkinematics.h"


void crkw_robot_kinematics(JOINTS& joints, FRAMES32& frames32){
    MatrixRow * frames_p = (MatrixRow *)frames32;
    FRAMES frames_m(frames_p, N_FRAMES, 1);
    new (&frames_m) FRAMES(&frames_p[0], N_FRAMES, 1);
    get_frames(frames_m, joints);
}

// gcc -O3 -std=c++1y -ffast-math -Ofast -fpermissive -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/ -c ./ccalibratedkinematics.cpp -o main.o

// g++ -c -Wall -Werror -fpic ccalibratedkinematics.cpp Justin19.cpp -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/
// g++ -shared -o libccalibratedkinematics.so Justin19.o ccalibratedkinematics.o -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/


//g++ -c -Wall -Werror -fpic ccalibratedkinematics.cpp Justin19.cpp -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/
//g++ -shared -o libccalibratedkinematics.so Justin19.o ccalibratedkinematics.o -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/

//g++ -Wall -o test main.cpp -lccalibratedkinematics.so -L/Users/jote/Documents/Code/Python/DLR/mopla/mopla/Kinematic/Robots/cpp/Justin19/ -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/

// /volume/USERSTORE/tenh_jo/Software/eigen-3.3.7
//g++ -c -Wall -Werror -fpic Justin19.cpp -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/
//g++ -shared -o libJustin19.so Justin19.o -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/
//g++ -Wall -o test main.cpp -I/Users/jote/Documents/Code/Software/C/eigen-3.3.7/ -lJustin19 -L/Users/jote/Documents/Code/Python/DLR/mopla/mopla/Kinematic/Robots/cpp/Justin19 -I/Users/jote/Documents/Code/Python/DLR/mopla/mopla/Kinematic/Robots/cpp/Justin19


//g++ -Wall -o test main.cpp -lJustin19 -L/home/tenh_jo/PycharmProjects/mopla/mopla/Kinematic/Robots/cpp/Justin19/ -I/home/tenh_jo/PycharmProjects/mopla/mopla/Kinematic/Robots/cpp/Justin19/ -I/volume/USERSTORE/tenh_jo/Software/eigen-3.3.7



// export LD_LIBRARY_PATH=/home/tenh_jo/PycharmProjects/mopla/mopla/Kinematic/Robots/cpp/Justin19/:$LD_LIBRARY_PATH