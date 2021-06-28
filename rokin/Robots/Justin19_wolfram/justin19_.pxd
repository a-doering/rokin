cdef extern from "ajustin/planner/crobotkinematics-wrapper.h":
    int crkw_init()
    void crkw_free()
    void crkw_robot_kinematics(double q[19],
                               double frames[32][4][4])
    void crkw_drobot_kinematics(double q[19],
                                double jacobi[19][32][4][4])
