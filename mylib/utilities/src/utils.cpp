#include "utils.hpp"
#include "math.h"

using namespace Eigen;

namespace utils {
double LPF1(double *u, double *y, double tau) {
  return (SAMPLING_TIME * (u[0] + u[1]) + (2 * tau - SAMPLING_TIME) * y[1]) /
         (2 * tau + SAMPLING_TIME);
}
// template <typename T> T LPF1(T *u, T *y, double tau) {
//   return (SAMPLING_TIME * (u[0] + u[1]) + (2 * tau - SAMPLING_TIME) * y[1]) /
//          (2 * tau + SAMPLING_TIME);
// }

// template <typename T> T LPF1(const T u_prev, const T u_current, const T y_prev, double tau) {
//   return (SAMPLING_TIME * (u_prev + u_current) + (2 * tau - SAMPLING_TIME) * y_prev) /
//          (2 * tau + SAMPLING_TIME);
// }

double LPF2(double *u, double *y, double tau) {
  return (pow(SAMPLING_TIME, 2) * u[0] + 2 * pow(SAMPLING_TIME, 2) * u[1] +
          pow(SAMPLING_TIME, 2) * u[2] - (2 * pow(SAMPLING_TIME, 2) - 8 * pow(tau, 2)) * y[1] -
          (pow(SAMPLING_TIME, 2) - 4 * SAMPLING_TIME * tau + 4 * pow(tau, 2)) * y[2]) /
         (pow(SAMPLING_TIME, 2) + 4 * SAMPLING_TIME * tau + 4 * pow(tau, 2));
}

double LPF4(double *u, double *y, double tau) {
  double a4 = pow(SAMPLING_TIME, 4);
  double a3 = 4 * pow(SAMPLING_TIME, 4);
  double a2 = 6 * pow(SAMPLING_TIME, 4);
  double a1 = 4 * pow(SAMPLING_TIME, 4);
  double a0 = pow(SAMPLING_TIME, 4);
  double b4 = (pow(SAMPLING_TIME, 4) + 8 * pow(SAMPLING_TIME, 3) * tau +
               24 * pow(SAMPLING_TIME, 2) * pow(tau, 2) + 32 * SAMPLING_TIME * pow(tau, 3) +
               16 * pow(tau, 4));
  double b3 = (4 * pow(SAMPLING_TIME, 4) + 16 * pow(SAMPLING_TIME, 3) * tau -
               64 * SAMPLING_TIME * pow(tau, 3) - 64 * pow(tau, 4));
  double b2 =
      (6 * pow(SAMPLING_TIME, 4) - 48 * pow(SAMPLING_TIME, 2) * pow(tau, 2) + 96 * pow(tau, 4));
  double b1 = (4 * pow(SAMPLING_TIME, 4) - 16 * pow(SAMPLING_TIME, 3) * tau +
               64 * SAMPLING_TIME * pow(tau, 3) - 64 * pow(tau, 4));
  double b0 = pow(SAMPLING_TIME, 4) - 8 * pow(SAMPLING_TIME, 3) * tau +
              24 * pow(SAMPLING_TIME, 2) * pow(tau, 2) - 32 * SAMPLING_TIME * pow(tau, 3) +
              16 * pow(tau, 4);
  double Num = a4 * u[4] + a3 * u[3] + a2 * u[2] + a1 * u[1] + a0 * u[0];
  double Den = b4 * y[4] + b3 * y[3] + b2 * y[2] + b1 * y[1];

  return (Num - Den) / b0;
}

// template <typename T> T diff1(T *u, T *y, double tau) {
//   return (-2 * u[1] + 2 * u[0] - (SAMPLING_TIME - 2 * tau) * y[1]) / (SAMPLING_TIME + 2 * tau);
// }

// template <typename T> T diff1(const T u_prev, const T u_current, const T y_prev, double tau) {
//   return (-2 * u_prev + 2 * u_current - (SAMPLING_TIME - 2 * tau) * y_prev) /
//          (SAMPLING_TIME + 2 * tau);
// }
VectorXd integrate(VectorXd *u, VectorXd *y) { return SAMPLING_TIME / 2 * (u[0] + u[1]) + y[1]; }

double InvMotorModel1(double *u, double *y, double Jm, double Bm, double tau) {
  return ((2 * Jm + Bm * SAMPLING_TIME) * u[0] + (Bm * SAMPLING_TIME - 2 * Jm) * u[1] -
          SAMPLING_TIME * y[1]) /
         SAMPLING_TIME;
}

double InvLoadAngleModel(double *u, double *y, double Jm, double Bm, double Jln, double Bl,
                         double K, int Nm, double tau) {
  double a4 = K * Nm * pow(SAMPLING_TIME, 4);
  double a3 = 4 * K * Nm * pow(SAMPLING_TIME, 4);
  double a2 = 6 * K * Nm * pow(SAMPLING_TIME, 4);
  double a1 = 4 * K * Nm * pow(SAMPLING_TIME, 4);
  double a0 = K * Nm * pow(SAMPLING_TIME, 4);
  double b4 =
      (2 * Bl * K * pow(SAMPLING_TIME, 3) + 16 * Jln * Jm * pow(Nm, 2) +
       4 * Jln * K * pow(SAMPLING_TIME, 2) + 4 * Bl * Bm * pow(Nm, 2) * pow(SAMPLING_TIME, 2) +
       2 * Bm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 3) +
       4 * Jm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 2) + 8 * Bl * Jm * pow(Nm, 2) * SAMPLING_TIME +
       8 * Bm * Jln * pow(Nm, 2) * SAMPLING_TIME);
  double b3 =
      (4 * Bl * K * pow(SAMPLING_TIME, 3) - 64 * Jln * Jm * pow(Nm, 2) +
       4 * Bm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 3) - 16 * Bl * Jm * pow(Nm, 2) * SAMPLING_TIME -
       16 * Bm * Jln * pow(Nm, 2) * SAMPLING_TIME);
  double b2 = (96 * Jln * Jm * pow(Nm, 2) - 8 * Jln * K * pow(SAMPLING_TIME, 2) -
               8 * Bl * Bm * pow(Nm, 2) * pow(SAMPLING_TIME, 2) -
               8 * Jm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 2));
  double b1 = (16 * Bl * Jm * pow(Nm, 2) * SAMPLING_TIME - 64 * Jln * Jm * pow(Nm, 2) -
               4 * Bm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 3) -
               4 * Bl * K * pow(SAMPLING_TIME, 3) + 16 * Bm * Jln * pow(Nm, 2) * SAMPLING_TIME);
  double b0 = 16 * Jln * Jm * pow(Nm, 2) - 2 * Bl * K * pow(SAMPLING_TIME, 3) +
              4 * Jln * K * pow(SAMPLING_TIME, 2) +
              4 * Bl * Bm * pow(Nm, 2) * pow(SAMPLING_TIME, 2) -
              2 * Bm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 3) +
              4 * Jm * K * pow(Nm, 2) * pow(SAMPLING_TIME, 2) -
              8 * Bl * Jm * pow(Nm, 2) * SAMPLING_TIME - 8 * Bm * Jln * pow(Nm, 2) * SAMPLING_TIME;
  double Num = a4 * y[4] + a3 * y[3] + a2 * y[2] + a1 * y[1];
  double Den = b4 * u[4] + b3 * u[3] + b2 * u[2] + b1 * u[1] + b0 * u[0];

  return (Den - Num) / a0;
}

Matrix3d omg2so3(Vector3d vec) {
  return (Eigen::Matrix<double, 3, 3>() << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1),
          vec(0), 0.0)
      .finished();
}

Vector3d so32omg(Matrix3d so3mat) {
  return (Eigen::Matrix<double, 3, 1>() << -so3mat(1, 2), so3mat(0, 2), -so3mat(0, 1)).finished();
}

Matrix3d MatrixLog(Matrix3d RotMat) {
  double acos_input = (RotMat.trace() - 1) / 2;
  double theta;
  Vector3d omg;
  Vector3d temp;
  Matrix3d so3mat;

  if (acos_input >= 1) { so3mat = Matrix3d::Zero(); }
  else if (acos_input <= -1) {
    if ((1 + RotMat(2, 2)) > 1e-6) {
      temp << RotMat(0, 2), RotMat(1, 2), 1 + RotMat(2, 2);
      omg = (1 / sqrt(2 * (1 + RotMat(1, 1)))) * temp;
    }
    if ((1 + RotMat(1, 1) > 1e-6)) {
      temp << 1 + RotMat(0, 0), RotMat(1, 0), RotMat(2, 0);
      omg = (1 / sqrt(2 * (1 + RotMat(2, 2)))) * temp;
    }
    omg *= M_PI;
    so3mat << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
  }
  else {
    theta = acos(acos_input);
    so3mat << theta * (1 / (2 * sin(theta))) * (RotMat - RotMat.transpose());
  }
  return so3mat;
}

Matrix3d MatrixExp(Matrix3d So3Mat) {
  //    double theta = acos((So3Mat.trace()-1)/2);
  double theta = utils::so32omg(So3Mat).norm();
  Matrix3d OmgHat(Matrix3d::Zero());
  if (theta < 1e-5)
    theta = 0.0;
  else
    OmgHat = So3Mat / theta;
  return Matrix3d::Identity() + sin(theta) * OmgHat + (1 - cos(theta)) * OmgHat * OmgHat;
}

MatrixXd NullProjection(MatrixXd J) {
  MatrixXd J_pinv = J.transpose() * (J * J.transpose()).fullPivLu().inverse();
  MatrixXd I = MatrixXd::Identity(7, 7);
  return I - J_pinv * J;
}

MatrixXd NullProjection(MatrixXd J, MatrixXd M) {
  //    MatrixXd J_pinv = M.inverse()*J.transpose()*(J*M.inverse()*J.transpose()).inverse();
  MatrixXd J_pinv = M.fullPivLu().inverse() * J.transpose() *
                    (J * M.fullPivLu().inverse() * J.transpose()).fullPivLu().inverse();
  MatrixXd I = MatrixXd::Identity(7, 7);
  return I - J_pinv * J;
}
} // namespace utils