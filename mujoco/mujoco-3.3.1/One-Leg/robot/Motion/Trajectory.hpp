#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

class Trajectory
{
public:
    Trajectory();
    ~Trajectory();
    
    Vector2d get_trajectory(double t, double z_vel, int Traj_mode);

private:    
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

};

