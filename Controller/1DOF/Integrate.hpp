#pragma once

#include <iostream>
#include <cmath>
#include <memory>
#include <Eigen/Core>
#include "process_shared_data.hpp"

using namespace Eigen;
using namespace std;

class Actuator;
class Trajectory;
class Controller;

class Integrate
{
public:
    Integrate(std::shared_ptr<Actuator> Act_,
              std::shared_ptr<Trajectory> Traj_,
            std::shared_ptr<Controller> Ctrl_

        );
            
    ~Integrate();

    void Cal_Kinematics(double t);
    void Ctrl();
    const Control_DATA& data() const;
    
    double get_joint_input();
    void get_state(double time);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

