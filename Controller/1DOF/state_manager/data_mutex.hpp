#ifndef DATA_MUTEX_H
#define DATA_MUTEX_H

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <inttypes.h> //
#include <array>
#include <vector>

#define NUMOFSLAVES 1 //Number of slaves
#define NUMOFSENSOR 1 //Number of slaves for sensor (Torque sensor)

#define NUMOFLEGS 1
struct SharedData_thread{
    
    pthread_mutex_t data_mut = PTHREAD_MUTEX_INITIALIZER;

    double sampling_time_ms;
    /* For ECAT */
    
    int overrun_cnt;
    std::array<uint16_t, NUMOFSLAVES+1> ecat_states{};
    
    int ecat_wkc = 0; //Actual Working Counter
    int ecat_expected_wkc = 0; //Working Counter which is expected.

    // 4) RT -> GUI/ROS (측정값/상태)
    std::array<double, NUMOFSLAVES> motor_position{};
    std::array<double, NUMOFSLAVES> motor_torque{};
    std::array<double, NUMOFSLAVES> motor_velocity{};
    
    std::array<double, NUMOFSLAVES> motor_position_raw_calc{};
    std::array<double, NUMOFSLAVES> Ain1{};
    std::array<uint8_t, NUMOFSLAVES> Din{};
    std::array<uint16_t, NUMOFSLAVES> statusword{};
    std::array<int8_t,  NUMOFSLAVES> mode_disp{};

    // 5) GUI/ROS -> RT (명령/설정)
    std::array<int8_t,    NUMOFSLAVES> mode_op{};
    std::array<uint16_t,  NUMOFSLAVES> controlword{};
    std::array<double,    NUMOFSLAVES> target_torque{};
    std::array<double,    NUMOFSLAVES> target_velocity{};
    std::array<double,    NUMOFSLAVES> target_position{};
    std::array<double,    NUMOFSLAVES> motor_offset{};
};

extern SharedData_thread d;


/* Mutex tools */
//
extern timespec data_mut_lock_timeout;
extern int timeout_ns;

/* Personal mutex */
extern double _M_Motor_offset[NUMOFSLAVES];

#endif // DATA_MUTEX_H
