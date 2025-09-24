#pragma once

#include <memory>
#include <Eigen/Core>


using namespace Eigen;

class Controller
{
public:
    Controller();
    ~Controller();                       // out-of-line dtor for pimpl

    // Run controller


    double joint_PID(const double& error, const double& error_old);

    void get_gain();
private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
