#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <cstdint>   // uint16_t, int8_t
#include <array>
#include <data_mutex.hpp>
#include "process_shared_data.hpp"


/**
 * @brief Actuator Class
 * @param target_torque: Load torque @param current: Motor current
 * 
 */
class Actuator
{
public:

    explicit Actuator(int numofslaves);
    ~Actuator();


    bool BindPDO();           // ← 추가: OP 전환 후 1회 호출

    void Receive_DATA();
    void Send_DATA();
    double get_joint_angle(int num);
    
    const Actuator_DATA& data() const;

    void resetMotorOffset();
    // double get_pos_load(int ACT_NUM);
    // double get_vel_load(int ACT_NUM);

private:
    
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
};

