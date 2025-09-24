#ifndef UTILS_H_
#define UTILS_H_

#include "Eigen/Core"
#include "Eigen/LU"
#include <cmath>
#include <cstdlib>
#include <inttypes.h>
#include <math.h>

#define SAMPLING_TIME 0.001 // 1 ms

namespace utils {

// template <typename T> T LPF1(T *u, T *y, double tau);
// template <typename T> T LPF1(const T u_prev, const T u_current, const T y_prev, double tau);
template <typename T> T LPF1(T *u, T *y, double tau) {
  return (SAMPLING_TIME * (u[0] + u[1]) + (2 * tau - SAMPLING_TIME) * y[1]) /
         (2 * tau + SAMPLING_TIME);
}

template <typename T> T LPF1(const T u_prev, const T u_current, const T y_prev, double tau) {
  return (SAMPLING_TIME * (u_prev + u_current) + (2 * tau - SAMPLING_TIME) * y_prev) /
         (2 * tau + SAMPLING_TIME);
}

template <typename T> T diff1(T *u, T *y, double tau) {
  return (-2 * u[1] + 2 * u[0] - (SAMPLING_TIME - 2 * tau) * y[1]) / (SAMPLING_TIME + 2 * tau);
}

template <typename T> T diff1(const T u_prev, const T u_current, const T y_prev, double tau) {
  return (-2 * u_prev + 2 * u_current - (SAMPLING_TIME - 2 * tau) * y_prev) /
         (SAMPLING_TIME + 2 * tau);
}
double LPF1_load(double *u, double *y, double tau);
double LPF2(double *u, double *y, double tau);
double LPF4(double *u, double *y, double tau);
Eigen::VectorXd integrate(Eigen::VectorXd *u, Eigen::VectorXd *y);

// u : input, y : output(including current and previous data)
// template <typename T> T diff1(T *u, T *y, double tau);
// // u_prev : previous input, u_current : current input, y_prev : previous output
// template <typename T> T diff1(const T u_prev, const T u_current, const T y_prev, double tau);

double InvMotorModel1(double *u, double *y, double Jm, double Bm, double tau);
double InvLoadAngleModel(double *u, double *y, double Jm, double Bm, double Jln, double Bl,
                         double K, int Nm, double tau);
Eigen::Matrix3d omg2so3(Eigen::Vector3d vec);
Eigen::Vector3d so32omg(Eigen::Matrix3d so3mat);
Eigen::Matrix3d MatrixLog(Eigen::Matrix3d RotMat);
Eigen::Matrix3d MatrixExp(Eigen::Matrix3d So3Mat);
Eigen::MatrixXd NullProjection(Eigen::MatrixXd J);
Eigen::MatrixXd NullProjection(Eigen::MatrixXd J, Eigen::MatrixXd M);
//    Eigen::MatrixXd CalcJdot(Eigen::MatrixXd* Jacobian){return (Jacobian[0]-Jacobian[0])/0.001;};

}; // namespace utils

#endif /* PID_H_ */
