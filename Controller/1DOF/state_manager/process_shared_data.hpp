#pragma once

#include <array>
#include <cstdint>       // uint16_t, int8_t
#include <data_mutex.hpp> // NUMOFSLAVES 
#include <atomic>

// 프로세스 간 공유 가능한 고정 크기 스냅샷
struct Actuator_DATA {
    uint32_t count{}; // 유효 슬레이브 수 (<= NUMOFSLAVES)
    std::array<double,   NUMOFSLAVES> motor_offset{};
    std::array<double,   NUMOFSLAVES> pos_raw{};
    std::array<double,   NUMOFSLAVES> joint_angle{};
    std::array<double,   NUMOFSLAVES> pos_refined{};
    std::array<double,   NUMOFSLAVES> position_raw_calc{};
    std::array<double,   NUMOFSLAVES> position_refined{};
    std::array<double,   NUMOFSLAVES> vel_raw{};
    std::array<double,   NUMOFSLAVES> vel_load{};
    std::array<double,   NUMOFSLAVES> torque_raw{};
    std::array<uint16_t, NUMOFSLAVES> statusword{};
    std::array<int8_t,   NUMOFSLAVES> modeofOP_disp{};
    std::array<double,   NUMOFSLAVES> current{};
    uint32_t numofslaves{}; 
};

struct GUI_Command {

    std::array<uint16_t, NUMOFSLAVES> controlword{};
    bool ctrl_on{};
    bool traj_on{};
    bool motor_offset_on{};
    // std::atomic<bool> apply_motor_offset{};

    std::array<double, NUMOFLEGS> kp_x, kp_y, kp_z;
    std::array<double, NUMOFLEGS> ki_x, ki_y, ki_z;
    std::array<double, NUMOFLEGS> kd_x, kd_y, kd_z;
    std::array<double, NUMOFLEGS> d_cutoff_x, d_cutoff_y, d_cutoff_z;
};

struct Control_DATA{

    // std::array<double, NUMOFSLAVES> des_leg_pos;
    // std::array<double, NUMOFSLAVES> leg_pos;
    // std::array<double, NUMOFSLAVES> des_leg_vel;
    // std::array<double, NUMOFSLAVES> leg_vel;

};