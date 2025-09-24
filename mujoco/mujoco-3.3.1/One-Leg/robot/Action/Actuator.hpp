#pragma once

#include <memory>
#include <mujoco/mjmodel.h>   
#include <mujoco/mjdata.h>    
#include "globals.hpp"        

class filter;

class Actuator {
public:
  Actuator(int numofjoints);
  ~Actuator();

  void Receive_data(const mjModel* m, mjData* d);
  double get_joint_angle(int num);

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};
