/* General includes */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> //C POSIX lib header. Header for accessing to the POSIX OS API
#include <fcntl.h> //C POSIX lib header. Header for opening and locking files and processing other tasks.
#include <signal.h> //Header for signal processing
#include <sys/timerfd.h> //
#include <string.h>
#include <malloc.h> //Memory allocation
#include <pthread.h> //Header for using Thread operation from xenomai pthread.h
#include <error.h> //
#include <errno.h> //Header for defining macros for reporting and retrieving error conditions using the symbol 'errno'
#include <sys/mman.h> //
// #include <rtdm/ipc.h> //
#include <inttypes.h> //
#include <iostream>   // std::cout, std::endl
#include <cstring>    // std::strerror
#include <memory>

/* x4 includes */
#include <evl/clock.h>
#include <evl/evl.h>
#include <evl/mutex.h>
#include <evl/syscall.h>
#include <evl/thread.h>
#include <evl/timer.h>

/* Personal includes */
//Have to write includes in CMakeLists.txt in order to fully
//include personal headers.
#include <data_mutex.hpp>
#include <ethercat.h>
#include <ecat_func.hpp>
#include "rt_func.hpp"
#include "intra_process_state_manager.hpp"
#include "inter_process_state_manager.hpp"
#include "process_shared_data.hpp"

#include "Controller.hpp"
#include "Actuator.hpp"
#include "Kinematics.hpp"
#include "Trajectory.hpp"
#include "Integrate.hpp"


//! Declare Variable for Control !//
bool ctrl_on = false;
bool traj_on = false;
bool motor_offset_on = false;

double cnt = 0;
double t = 0;


//! Declare Shared Pointer !//

    std::shared_ptr<rt_func> rt_func_;
    
    auto Act_ = std::make_shared<Actuator>(NUMOFSLAVES);
    auto Traj_ = std::make_shared<Trajectory>();
    auto Ctrl_ = std::make_shared<Controller>();
    auto Kin_ = std::make_shared<Kinematics>(Act_ ,Traj_);

    auto I_ = std::make_shared<Integrate>(Act_, Traj_, Ctrl_, Kin_);
    
//! Mutex Data !//
    SharedData_thread d{};

using namespace std;

//personal
double motor_offset[NUMOFSLAVES];       
double position_refined[NUMOFSLAVES];   
double position_raw_calc[NUMOFSLAVES];  
//SEND                                  
int8_t modeOP[NUMOFSLAVES];             
uint16_t controlword[NUMOFSLAVES];      
double target_torque[NUMOFSLAVES];      
double target_speed[NUMOFSLAVES];       
double target_position[NUMOFSLAVES];    
uint32_t digital_output[NUMOFSLAVES];   
//RECEIVE                               
int32_t position_raw[NUMOFSLAVES];     
int32_t velocity_raw[NUMOFSLAVES];     
int16_t torque_raw[NUMOFSLAVES];       
int16_t Ain1_raw[NUMOFSLAVES];          
// uint32_t DCvolt_raw[NUMOFSLAVES];       
uint32_t Din_raw[NUMOFSLAVES];          
uint16_t statusword[NUMOFSLAVES];      
int8_t modeofOP_disp[NUMOFSLAVES];     
uint16_t slave_state[NUMOFSLAVES];     




/******************************/
/******* Shared Memory ********/
/******************************/

InterProcessStateManager *shm_manager = nullptr;


/* Setup for RT thread */
#define RT_PERIOD_MS 1 // 1 msec 실시간 루프 주기
#define XDDP_PORT 0 // RT 파이프 포트 번호

pthread_t rt;
int sigRTthreadKill = 0;
//End of setup for RT thread


/* Definitions for SOEM */
const char *IFNAME = "enp2s0"; // "enx98fc84ec7fcc" "enp1s0"

char IOmap[4096]; //

int expectedWKC; //Working Counter:
volatile int wkc; //Doesn't want to occupy memory location

/* Definitions for CiA402 (DS402?) */
double CL[NUMOFSLAVES]; //Current Limit(CL) in ELMO driver, in Amp

Vector2d pos_error = Vector2d::Zero();
Vector2d pos_error_old = Vector2d::Zero();
Vector2d pos_ref = Vector2d::Zero();
Vector2d pos_act = Vector2d::Zero();

Vector2d q_act = Vector2d::Zero();
Matrix2d jacobian = Matrix2d::Zero();

// Thread work functions
auto main_thread = [](double sampling_ms, unsigned long long overrun_count){
    static int tick=0;

    t = 0.001*cnt;

    if(!sigRTthreadKill) //perform the task untill the kill signal is reached
    {
        ec_send_processdata(); //PDO sending

        Act_->Receive_DATA();

        I_->get_state(t);

    //!User codes for RT task (Control algorithms)
    
        I_->Cal_Kinematics(t);

        for(int i = 0; i < NUMOFSLAVES; i++ )
            target_torque[i] = 0;

        if(ctrl_on){
        
        
            
            
            I_->Ctrl();

                

            for(int i = 0; i< 2; i++)
                target_torque[i] = I_->get_joint_input()[i];

                // cout <<"target torque: " << target_torque[0] << "  " << target_torque[1] << endl;
            
            //! Debug
                        

            // double Kp = 15;
            // double f = 1;
            // // double err = 1*sin(2*M_PI*f*t) + 3*M_PI/4 - Act_->get_joint_angle(1);
            // double err = 0.1*t + 3*M_PI/4 - Act_->get_joint_angle(1);

            // target_torque[0] = 0;
            // target_torque[1] = Kp * err;    

            // cout << " torque: "<< target_torque[1] << endl;
            
            //! Debug
        }
    
        //! Multi Processing
        {
            auto& shm = InterProcessStateManager::getInstance(); // 이미 생성된 싱글턴
            auto acc = shm.getSafeAccess();                      // lock 자동
            
            for(int i = 0; i < NUMOFSLAVES; i++)
            {
                
                //! Send to GUI
                    acc->control = I_->data();     
                    
                    
                //* Send Actuator Data
                    acc->actuator = Act_->data();
                
                
                //? Get from GUI
                    //* Control Word *//
                        d.controlword[i] = acc->gui_cmd.controlword[i];
                        
                    //* Switch Signal *//
                        ctrl_on = acc->gui_cmd.ctrl_on;
                        traj_on = acc->gui_cmd.traj_on;
                        // motor_offset_on = acc->gui_cmd.motor_offset;
                        // cout << acc->gui_cmd.motor_offset_on << endl;


                        if(acc->gui_cmd.motor_offset_on)
                        {
                            Act_->resetMotorOffset();
                        }


                // cout << i << ": " << acc->gui_cmd.controlword[i] << endl;
                // cout << acc->actuator.joint_angle[0] << endl;
                
            }
             acc->b_system_ready = true;

            // 필요하면 같은 락 스코프에서 목표값을 읽어 제어에 반영할 수도 있음:
            // double q_ref = acc->joint_pos_target[0];
        } // unlock 자동

        
        
        
        
        //Try to get mutex to sync data with GUI thread
        if(!pthread_mutex_trylock(&d.data_mut)) //trylock: Doesn't wait lock.
        {
            // Have to study this part
            d.sampling_time_ms = sampling_ms; //when the thread get the mutex, write data into shared global variables
            d.overrun_cnt = overrun_count;

            d.ecat_wkc = wkc;
            d.ecat_expected_wkc = expectedWKC;

            d.ecat_states[0] = slave_state[0]; //
            for (int i=0;i<NUMOFSLAVES;i++)
            {
                d.motor_position[i]            = position_refined[i];
                d.motor_position_raw_calc[i]   = position_raw_calc[i];
                d.motor_torque[i]              = ((double)torque_raw[i]) * (20.0 / 1000.0);
                // cout << ((double)torque_raw[i]) * (20.0 / 1000.0) << endl;
                d.motor_velocity[i]            = (double)velocity_raw[i] / (360.0 / 1048576.0 / 9.0);
                d.Ain1[i]                      = (double)Ain1_raw[i]*0.001;
                d.Din[i]                       = (uint8_t)(Din_raw[i]>>16);
                d.statusword[i]                = statusword[i];
                d.mode_disp[i] = modeofOP_disp[i];
                d.ecat_states[i+1] = slave_state[i+1]; //Since 0:Master


                modeOP[i]           = 10;
                
                controlword[i]       = d.controlword[i];
                target_speed[i]     = d.target_velocity[i];
                target_position[i]  = d.target_position[i];

                motor_offset[i]     = d.motor_offset[i];

                d.target_torque[i]    = target_torque[i];

                // read_error_register_with_sdo(i + 1);

            }

            Act_->Send_DATA();

            pthread_mutex_unlock(&d.data_mut); //after writing data, unlock mutex ASAP
        }

            if(traj_on) cnt++;
    }


};
auto pinocchio_thread = [](double sampling_ms, unsigned long long overrun_count){
  static int tick=0;
  // std::cout << "[B] tick " << tick++ << std::endl;
};
auto logging_thread = [](double sampling_ms, unsigned long long overrun_count){
  static int tick=0;
  // std::cout << "[C] tick " << tick++ << std::endl;
};

// Thread entry wrappers
void *threadA(void *) 
{
    // SOEM 초기화
    if (!soem_init()) {
        std::cerr << "SOEM init failed\n";
        return nullptr;
    }
    if (!Act_->BindPDO()) {
    std::cerr << "BindPDO failed for Actuator\n";
    return nullptr;
    }
    else
    std::cerr << "Binding is done successfully!\n";

    rt_func_->run_periodic(0, 1, 90, main_thread); // cpu num, sampling time, RT priority(높으면 더 우선)
    return nullptr; 
}
void *threadB(void *) { rt_func_->run_periodic(1, 2, 80, pinocchio_thread); return nullptr; }
void *threadC(void *) { rt_func_->run_periodic(2, 3, 70, logging_thread); return nullptr; }



/* Main Function */
int main(int argc, char *argv[])
{
    //2) 변수 초기화
    //RxPDO
    d.motor_position.fill(0.0);
    d.motor_torque.fill(0.0);
    d.motor_velocity.fill(0.0);    
    d.Ain1.fill(0.0);
    d.Din.fill(0.0);
    d.statusword.fill(0.0);
    d.mode_disp.fill(0.0);
    //TxPDO
    d.mode_op.fill(0.0);
    d.controlword.fill(0.0);
    d.target_torque.fill(0.0);    
    d.ecat_states.fill(0.0);
    d.motor_offset.fill(0.0);
    d.motor_position_raw_calc.fill(0.0);

    // Shared Memory 생성 및 초기화
        auto& shm = InterProcessStateManager::getInstance(/*creator=*/true); // 생성자 프로세스
        {
        auto acc = shm.getSafeAccess(); // lock 자동
        
        cout << "System is ready" << endl;
        acc->b_system_ready = true;
        } // unlock 자동
        
    
        //3) 구동기 전류 제한(CL) 기본값 설정 (슬레이브 수만큼 20A)
    std::fill_n(CL, NUMOFSLAVES, 20);

    // 워커 스레드 3개 생성: 
    constexpr int NUM_THREADS = 3;
    pthread_t threads[NUM_THREADS];

    // create threads with distinct functions and pin to cores
    pthread_create(&threads[0], nullptr, threadA, nullptr);
    rt_func_->pin_to_cpu(threads[0], 0);  
    pthread_create(&threads[1], nullptr, threadB, nullptr);
    rt_func_->pin_to_cpu(threads[1], 1);
    pthread_create(&threads[2], nullptr, threadC, nullptr);
    rt_func_->pin_to_cpu(threads[2], 2);

    // GUI 
    // w_Main.show(); ############################
    // int ret = a.exec();  // ############################


    
    // Thread 종료
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], nullptr);
    }

    // return ret;
}
