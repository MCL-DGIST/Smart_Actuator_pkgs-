// filter.hpp
#pragma once

#include <memory>
#include "Eigen/Core"

using namespace Eigen;

class filter {
public:
    filter();
    ~filter();

    double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    Vector3d quat2euler(double qw, double qx, double qy, double qz);
    Vector3d quat2rpy(const Vector4d &quat);
    Matrix3d skew(const Vector3d &v);

private:
    struct Impl;                    // 전방 선언
    std::unique_ptr<Impl> pimpl_;   // 실제 구현은 Impl 내부에
};
