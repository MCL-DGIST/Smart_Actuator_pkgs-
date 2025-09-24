#pragma once
#include <string>
#include <vector>

struct ActuatorParams {
  std::vector<double> home_angle, stiffness;
  std::vector<double> motor_inertia, motor_damping, load_inertia, load_damping;
  std::vector<double> max_load_angle, min_load_angle;
};

// yaml_path로부터 ActuatorParams 로드
ActuatorParams loadActuatorParams(const std::string &yaml_path);
