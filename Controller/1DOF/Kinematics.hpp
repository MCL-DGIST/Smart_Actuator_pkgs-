#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>


using namespace Eigen;
class Actuator;
class Trajectory;
class Kinematics
{
public:
    Kinematics(std::shared_ptr<Actuator> Act_,
              std::shared_ptr<Trajectory> Traj_);
    ~Kinematics();                       // out-of-line dtor for pimpl
    
    void Cal_kinematics(double t);
    
    Vector2d get_pos();
    Vector2d get_pos_err();
    Vector2d get_pos_err_old();
    Matrix2d get_jacobian();

    
private:

    struct Impl;
    std::unique_ptr<Impl> pimpl_;

};
