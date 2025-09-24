#pragma once

#include <iostream>
#include <cmath>
#include <memory>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

class Actuator;
class Trajectory;
class Controller;
class Kinematics;
class Sensordata;

class Integrate
{
public:
    Integrate(std::shared_ptr<Actuator> Act_,
              std::shared_ptr<Trajectory> Traj_,
            std::shared_ptr<Controller> Ctrl_,
            std::shared_ptr<Kinematics> Kin_,
            std::shared_ptr<Sensordata> Sen_
            
        );
            
    ~Integrate();

    void Cal_Kinematics(double t);
    void Ctrl();
    
    Vector2d get_joint_input();
    void get_state(double time);

    struct Logging_data
    {
        double t = 0;
        Vector2d pos = Vector2d::Zero();
        Vector2d pos_ref = Vector2d::Zero();
        Vector2d vel = Vector2d::Zero();
        Vector2d vel_ref = Vector2d::Zero();
        Vector2d j_pos = Vector2d::Zero();
        Vector2d j_pos_ref = Vector2d::Zero();
        double z_vel = 0;
    };

    Logging_data get_logging_data();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

