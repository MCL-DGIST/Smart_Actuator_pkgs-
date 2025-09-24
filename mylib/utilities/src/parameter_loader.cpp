#include "parameter_loader.hpp"
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {
template <typename T = double> std::vector<T> asVector(const YAML::Node &n, const char *key) {
  if (!n[key]) throw std::runtime_error(std::string("Missing key: ") + key);
  const auto seq = n[key];
  if (!seq.IsSequence()) throw std::runtime_error(std::string("Key not a sequence: ") + key);
  std::vector<T> out;
  out.reserve(seq.size());
  for (auto v : seq) out.push_back(v.as<T>());
  return out;
}
} // namespace

ActuatorParams loadActuatorParams(const std::string &yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root || !root["ActuatorParams"])
    throw std::runtime_error("YAML: top-level 'ActuatorParams' not found");

  const auto &p = root["ActuatorParams"];
  ActuatorParams ap;
  ap.home_angle = asVector(p, "home_angle");
  ap.stiffness = asVector(p, "stiffness");
  ap.motor_inertia = asVector(p, "motor_inertia");
  ap.motor_damping = asVector(p, "motor_damping");
  ap.load_inertia = asVector(p, "load_inertia");
  ap.load_damping = asVector(p, "load_damping");
  ap.max_load_angle = asVector(p, "max_load_angle");
  ap.min_load_angle = asVector(p, "min_load_angle");
  return ap;
}
