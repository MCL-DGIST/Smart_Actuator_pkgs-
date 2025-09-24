// Actuator.cpp
#include "Actuator.hpp"
#include "filter.hpp"
#include <vector>

struct Actuator::Impl {
  filter* F;
  int     numofjoints;
  double  d_cutoff;

  std::vector<double> q, q_old;
  std::vector<double> qd_tustin, qd_tustin_old;
  std::vector<double> qdd_tustin, qdd_tustin_old;

  Impl(int n)
    : F(nullptr),
      numofjoints(n),
      d_cutoff(100.0),
      q(n, 0.0), q_old(n, 0.0),
      qd_tustin(n, 0.0), qd_tustin_old(n, 0.0),
      qdd_tustin(n, 0.0), qdd_tustin_old(n, 0.0)
  {}

  void Receive_data(const mjModel* m, mjData* d) {
    for (int i = 0; i < numofjoints; ++i) {
      q[i] = d->qpos[1+i];
    }
      //* Biarticular
      q[1] = q[0] + q[1];

    for (int i = 0; i < numofjoints; ++i) {
      
      double vel   = F->tustin_derivative(q[i],      q_old[i],      qd_tustin_old[i], d_cutoff);
      double accel = F->tustin_derivative(vel,       qd_tustin_old[i], qdd_tustin_old[i], d_cutoff);

      q_old[i]          = q[i];
      qd_tustin_old[i]  = vel;
      qdd_tustin_old[i] = accel;

      // 최신 값 저장
      qd_tustin[i]  = vel;
      qdd_tustin[i] = accel;
    }

  }
};

Actuator::Actuator(int n)
  : pimpl_(std::make_unique<Impl>(n))
{}

Actuator::~Actuator() = default;

void Actuator::Receive_data(const mjModel* m, mjData* d) {
  pimpl_->Receive_data(m, d);
}

double Actuator::get_joint_angle(int num) {
  return pimpl_->q[num];
}
