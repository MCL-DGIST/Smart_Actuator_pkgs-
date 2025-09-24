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
    Trajectory(/* args */);
    ~Trajectory();

    Vector2d swing_traj(double t);

private:    
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

};

