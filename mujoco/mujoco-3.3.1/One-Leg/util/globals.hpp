#pragma once

#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <numeric>

    using namespace std;
    using namespace Eigen;



#define NUMOFJOINTS 2



extern const double Ts; // sampling period
extern const double g;    // gravitational accel.


struct controlparams {
    double kp;
    double ki;
    double kd;
};


extern struct controlparams CP;
