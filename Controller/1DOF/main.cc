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


            #include "ecat_device_control_objects.hpp"
            #include "ecat_hardware_interface.hpp"

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

    auto I_ = std::make_shared<Integrate>(Act_, Traj_, Ctrl_);
    
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
const char *IFNAME = "enp1s0"; // "enx98fc84ec7fcc" "enp1s0"

char IOmap[4096]; //

int expectedWKC; //Working Counter:
volatile int wkc; //Doesn't want to occupy memory location

/* Definitions for CiA402 (DS402?) */
double CL[NUMOFSLAVES]; //Current Limit(CL) in ELMO driver, in Amp


//! Debugging



        #define RT_THREAD_NUM 4         // EtherCAT (0), ELMO R/W (1), Control (2)
        #define RT_DYNAMICS_PERIOD_MS 1
        #define RT_ELMO_PERIOD_MS 1     // Real-time ELMO PDO Read/Write period (1 msec)
        #define RT_CTRL_PERIOD_MS 1     // Real-time control period (1 msec)
        #define RT_DATALOG_MS 100       // Real-time control period (1 msec)



        /* Event flags for threads synchronization */
        static std::atomic<bool> ThreadRunningFlag{true};

        #ifdef SLAVE_GTWI                           // * this is defined in ecat_hardware_interface.h
            GtwiRxPDOMap *elmo_gtwi_rx_pdo[SLAVES_NUM]; // RxPDO mapping data (output data into slaves)
            GtwiTxPDOMap *elmo_tx_pdo[SLAVES_NUM];      // TxPDO mapping data (input data from slaves)
        #endif
        
        #ifdef SLAVE_PTWI                           // * this is defined in ecat_hardware_interface.h
            PtwiRxPDOMap *elmo_ptwi_rx_pdo[SLAVES_NUM]; // RxPDO mapping data (output data into slaves)
            PtwiTxPDOMap *elmo_ptwi_tx_pdo[SLAVES_NUM]; // TxPDO mapping data (input data from slaves)
        #endif

        #ifdef SLAVE_ANYBUS
            AnybusRxPDOMap *anybus_rx_pdo; // RxPDO mapping data (input data from slaves)
            AnybusTxPDOMap *anybus_tx_pdo; // TxPDO mapping data (output data into slaves)
        #endif

        static void add_us(struct timespec &dst, const struct timespec &src, long us) {
        // 1. 초(second) 단위 더하기
        dst.tv_sec = src.tv_sec + us / 1000000;
        // 2. 나노초(nanosecond) 단위 더하기 (1us = 1000ns)
        dst.tv_nsec = src.tv_nsec + (us % 1000000) * 1000;

        // 3. 나노초 자리올림 처리 (1초 = 1,000,000,000 나노초)
        if (dst.tv_nsec >= 1000000000L) {
            dst.tv_sec++;
            dst.tv_nsec -= 1000000000L;
        }
        }

        template <typename Work> void run_periodic(int id, int period_ms, Work work) {
        // per‐thread timing state
        struct timespec prev_rt = {0, 0};
        unsigned long long overrun_count = 0;

        // attach to EVL core
        int desc = evl_attach_self("rt_thread_%d:%d", id, getpid());
        if (desc < 0) {
            std::cerr << "evl_attach_self(" << id << ") failed: " << std::strerror(-desc) << std::endl;
            return;
        }

        // create EVL timer
        int tfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
        if (tfd < 0) {
            std::cerr << "evl_new_timer(" << id << ") failed: " << std::strerror(-tfd) << std::endl;
            return;
        }

        // configure periodic timer
        struct timespec now;
        evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
        // struct itimerspec its{};
        // add_us(its.it_value, now, period_us);
        // its.it_interval.tv_sec = period_us / 1000000;
        // its.it_interval.tv_nsec = (period_us % 1000000) * 1000L;

    //! Debugging
        struct itimerspec its{};
        int period_us = period_ms * 1000;
        if (period_us < 100) period_us = 100; // 안전가드: 100 μs 미만 금지
        add_us(its.it_value, now, period_us);
        its.it_interval.tv_sec  = period_us / 1000000;
        its.it_interval.tv_nsec = (period_us % 1000000) * 1000L;

        if (evl_set_timer(tfd, &its, nullptr) < 0) {
            std::cerr << "evl_set_timer(" << id << ") failed" << std::endl;
            return;
        }

        while (ThreadRunningFlag) {
            __u64 ticks;
            if (oob_read(tfd, &ticks, sizeof(ticks)) < 0) {
            std::cerr << "oob_read(" << id << ") failed: " << std::strerror(-ticks) << std::endl;
            //            evl_printf("oob_read(%d) failed: %s\n",
            //            id,std::strerror(-ticks));
            break;
            }

            // 1) 시스템 시각 수집
            struct timespec trt;
            //        clock_gettime(CLOCK_REALTIME, &trt);
            evl_read_clock(EVL_CLOCK_MONOTONIC, &trt);

            // 2) delta 계산 (ns 단위)
            long delta_ns;
            if (prev_rt.tv_sec == 0 && prev_rt.tv_nsec == 0) {
            // 첫 루프인 경우
            delta_ns = period_us * 1000L;
            }
            else {
            delta_ns = (trt.tv_sec - prev_rt.tv_sec) * 1000000000L + (trt.tv_nsec - prev_rt.tv_nsec);
            }
            prev_rt = trt;

            // 3) ms 단위 변환
            double sampling_ms = delta_ns * 1e-6;

            // 4) jitter 계산
            // double jitter_ms = sampling_ms - static_cast<double>(period_us / 1000.0);
        //! Debugging
            double jitter_ms = sampling_ms - static_cast<double>(period_ms);
            // 5) overrun 집계
            if (ticks > 1) { overrun_count += (ticks - 1); }

            work(sampling_ms, overrun_count); // call user-provided work every cycle
        }
        }

        auto slaveReadWriteLoop = [](double sampling_ms, unsigned long long overrun_count) {
        // 11-2. Sending processdata (calculated one tick before) => In order to ensure punctuality

        ec_send_overlap_processdata(); // PDO sending

        wkc = ec_receive_processdata(EC_TIMEOUTRET); // returns WKC

        if (expectedWKC > wkc) {
            // In case of checked wkc less than WKC that have to be right
            // This means the etherCAT frame cannot be successfully read or wrote on at least one
            // slaves.
            ThreadRunningFlag = 0; // Kill the entire of the RT thread
            cout << "sigRTKilled at wkc calculation!" << endl;
        }

        // for (int i = 0; i < SLAVES_NUM; ++i) {
        //     actuators_ptr[i].receiveData();   // Receive data from slaves
        //     actuators_ptr[i].processStates(); // Process the states of the actuators
        //     actuators_ptr[i].sendData();      // Send data to slaves
        // }
        };

        void *slaveReadWriteThread(void *arg) {
            // 1. Initialize EtherCAT Master
            if (!initEcatMaster(IFNAME)) {
                std::cerr << "Error: ecat_init failed on interface " << IFNAME << "\n";
                ThreadRunningFlag = false;
            }
            ThreadRunningFlag = true;
            // 2. EtherCAT slave number check

            #ifdef NON_SLAVE_TS
            if (ec_slavecount == SLAVES_NUM) {
                ThreadRunningFlag = 0;
                // for(int i=1;i<=SLAVES_NUM-1;i++)
            
                #ifdef SLAVE_GTWI
                for (int i = 1; i <= NUMOF_GTWI_SLAVES; i++) {
                // PO: Pre-Operation, SO: Safe-Operation
                // Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_GTWI; // Doesn't this
                function need argument ?
                //=> PO2SOconfig is also function.
                }
            #endif
            #ifdef SLAVE_PTWI
                for (int i = NUMOF_GTWI_SLAVES + 1; i <= NUMOF_GTWI_SLAVES + PTWI_SLAVES_NUM; i++) {
                // PO: Pre-Operation, SO: Safe-Operation
                // Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_PTWI; // Doesn't this
                function need argument ?
                //=> PO2SOconfig is also function.
                }
            #endif
            }
            else
                std::cout << "sigRTKilled at ecat_init!" << endl;
            return false;
            #endif

            #ifdef SLAVE_ANYBUS

                if (ec_slavecount == SLAVES_NUM + 1) {
                    ThreadRunningFlag = true;
                    for (int i = 1; i <= SLAVES_NUM + 1; i++) {
                    // PO: Pre-Operation, SO: Safe-Operation
                    // Link slave's specific setups to PO -> SO
                    if (i == 1) { ec_slave[i].PO2SOconfig = ecat_PDO_Config_TS; }

                    else if (i > 1 && i <= PTWI_SLAVES_NUM + 1) {
                #ifdef SLAVE_PTWI
                        ec_slave[i].PO2SOconfig = ecat_PDO_Config_PTWI; // Doesn't this function need argument ?
                #endif
                    }
                    else {

                #ifdef SLAVE_GTWI
                        ec_slave[i].PO2SOconfig = ecat_PDO_Config_GTWI; // Doesn't this function need argument ?
                #endif
                    }
                    //=> PO2SOconfig is also function.
                    }
                }
                else {
                    std::cout << "AAAAAAAAAAAAAAAa" << endl;
                    std::cout << "sigRTKilled at PDO mapping!" << endl;
                    ThreadRunningFlag = false;
                }
            #endif



            ec_config_overlap_map(
                &IOmap); // Map all PDOs from slaves to IOmap with Outputs / Inputs in sequential order.

            // 3. Setting Distributed Clock
            if (ec_configdc() > 0) {
                // our control period is 1 ms → 1,000,000 ns
                uint32 cycletime_ns = 1000 * 1000UL;
                for (int i = 1; i <= ec_slavecount; ++i) {
                if (ec_slave[i].hasdc) {
                    // activate SYNC0 on each DC‐capable slave
                    ec_dcsync0(i, TRUE, cycletime_ns, 0);
                }
                }
            }

            ec_statecheck(0, EC_STATE_SAFE_OP, 4 * EC_TIMEOUTSTATE); // EC_TIMEOUTSTATE=2,000,000 us

            // 6. Calculate expected WKC
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC; // Calculate WKCs

            // 7. Change MASTER state to operational
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            // 8. Send one valid process data(Execute PDO communication once)
            ec_send_overlap_processdata();               // Send processdata to slave.
            wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive processdata from slaves.

            // 9. Request OP state for all slaves
            ec_writestate(0); // Write slave state.

            // 10. PDO data Receive/Transmit
            uint32 obytes = ec_slave[1].Obytes; // Obytes: Output bytes
            uint32 ibytes = ec_slave[1].Ibytes; // Ibytes: Input bytes

            for (int i = 0; i < SLAVES_NUM; ++i) {
            #ifdef SLAVE_GTWI
                elmo_gtwi_rx_pdo[i] = (GtwiRxPDOMap *)ec_slave[i + 1].outputs; // ! ec_slave[0] is ECAT master
                elmo_gtwi_tx_pdo[i] = (GtwiTxPDOMap *)ec_slave[i + 1].inputs;  // ! while it is not for elmo
            #endif
            #ifdef SLAVE_PTWI
                elmo_ptwi_rx_pdo[i] =
                    (PtwiRxPDOMap *)ec_slave[i + 2]
                        .outputs; // ! ec_slave[0] is ECAT master, ec_slave[1] is ANYBUS communicator
                elmo_ptwi_tx_pdo[i] = (PtwiTxPDOMap *)ec_slave[i + 2].inputs; // ! while it is not for elmo
            #endif
            }
            #ifdef SLAVE_ANYBUS
            anybus_rx_pdo = (AnybusRxPDOMap *)ec_slave[1]
                                .outputs; // ! ec_slave[0] is ECAT master, ec_slave[1] is ANYBUS communicator
            anybus_tx_pdo = (AnybusTxPDOMap *)ec_slave[1].inputs;
            #endif
            for (int i = 0; i < SLAVES_NUM; ++i) {
            #ifdef SLAVE_PTWI
                Act_->setPtwiPdoPointers(elmo_ptwi_tx_pdo[i], elmo_ptwi_rx_pdo[i]);
            #endif
            #ifdef SLAVE_GTWI
                Act_->setGtwiPdoPointers(elmo_gtwi_tx_pdo[i], elmo_gtwi_rx_pdo[i]);
            #endif
            #ifdef SLAVE_ANYBUS
                Act_->setAnybusPdoPointers(anybus_tx_pdo);
            #endif
            }

            run_periodic(0, RT_ELMO_PERIOD_MS, slaveReadWriteLoop);

            return nullptr; // Thread will exit after this
        }


//! Debugging

        void *controlThread(void *arg) {

    auto main_work = [](double sampling_ms, unsigned long long overrun_count) {
        // cout << "Main thread running with sampling time: " << sampling_ms
        //      << " ms, overrun count: " << overrun_count << endl;

        //     pController->setDelayData();
        //     for (int i = 0; i < SLAVES_NUM; ++i)
        //         pController->ACT[i].ReceiveData();

        //     pController->control_loop();

        //     for (int i = 0; i < SLAVES_NUM; ++i)
        //         pController->ACT[i].SendData();

        //     if(!pthread_mutex_trylock(&data_mut))
        //     {

        //         pController->Mutex_exchange();

        //         _M_sampling_time_ms = sampling_ms; //when the thread get the
        //         mutex, write data into shared global variables _M_overrun_cnt =
        //         overrun_count;

        //         _M_Ecat_WKC = wkc;
        //         _M_Ecat_expectedWKC = expectedWKC;
        //         // main_thread 내부, Mutex 교환 후
        //         //        printf("J0 SW: 0x%04X, CW: %u\n", _M_STATUSWORD[0],
        //         _M_CONTROLWORD[0]); pthread_mutex_unlock(&data_mut);

        //     }
    };

    run_periodic(0, 500, main_work);
    return nullptr;
    }
    void *calculateDynamicsThread(void *arg) {
    // // Manipulator* pMani_RA = static_cast<Manipulator*>(arg);
    auto dynamicsLoop = [](double sampling_ms, unsigned long long overrun_count) {
    //     manipulator_ptr->update_model();
    //     manipulator_ptr->Calc_Kinematics("end_effector");
    //     manipulator_ptr->Calc_Kinematics("Link4");
    //     manipulator_ptr->Calc_Kinematics("Link6");
    };

    run_periodic(1, RT_DYNAMICS_PERIOD_MS, dynamicsLoop);
    return nullptr;
    }


    int spawn_rt_thread(pthread_t *out_thread, int cpu, int priority, void *(*entry)(void *),
                    void *arg = nullptr, int policy = SCHED_FIFO, bool explicit_sched = true) {
        pthread_attr_t attr;
        struct sched_param sch;
        cpu_set_t cpus;
        int ret = 0;

        ret = pthread_attr_init(&attr);
        if (ret) {
            std::fprintf(stderr, "attr_init: %d\n", ret);
            return ret;
        }

        if (explicit_sched) {
            ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
            if (ret) {
            std::fprintf(stderr, "setinheritsched: %d\n", ret);
            goto CLEANUP;
            }
        }

        ret = pthread_attr_setschedpolicy(&attr, policy);
        if (ret) {
            std::fprintf(stderr, "setschedpolicy: %d\n", ret);
            goto CLEANUP;
        }

        sch.sched_priority = priority;
        ret = pthread_attr_setschedparam(&attr, &sch);
        if (ret) {
            std::fprintf(stderr, "setschedparam: %d\n", ret);
            goto CLEANUP;
        }

        CPU_ZERO(&cpus);
        CPU_SET(cpu, &cpus);
        ret = pthread_attr_setaffinity_np(&attr, sizeof(cpus), &cpus);
        if (ret) {
            std::fprintf(stderr, "setaffinity: %d\n", ret);
            goto CLEANUP;
        }

        ret = pthread_create(out_thread, &attr, entry, arg);
        if (ret) { std::fprintf(stderr, "pthread_create: %d\n", ret); }

        CLEANUP:
        pthread_attr_destroy(&attr);
        return ret;
    }



// Thread work functions
auto main_thread = [](double sampling_ms, unsigned long long overrun_count){
    static int tick=0;

    t = 0.001*cnt;

    if(!sigRTthreadKill) //perform the task untill the kill signal is reached
    {

        ec_send_overlap_processdata();
        // ec_send_processdata(); //PDO sending
        
        wkc = ec_receive_processdata(EC_TIMEOUTRET); // returns WKC

        if (expectedWKC > wkc) {
            // In case of checked wkc less than WKC that have to be right
            // This means the etherCAT frame cannot be successfully read or wrote on at least one
            // slaves.
            ThreadRunningFlag = 0; // Kill the entire of the RT thread
            cout << "sigRTKilled at wkc calculation!" << endl;
        }
        
        Act_->Receive_DATA();

        I_->get_state(t);

    //!User codes for RT task (Control algorithms)
    
    
        for(int i = 0; i < NUMOFSLAVES; i++ )
            target_torque[i] = 0;

        if(ctrl_on){
        
        
            
            
            I_->Ctrl();

                
            
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
    // constexpr int NUM_THREADS = 3;
    // pthread_t threads[NUM_THREADS];
    
    
      // Create threads for EtherCAT, ELMO R/W, Control, and Pinocchio dynamics calculation
    constexpr int NUM_THREADS = 3;

    constexpr int CPU[NUM_THREADS] = {0, 1, 2};         // CPU affinity for each thread
    constexpr int PRIORITY[NUM_THREADS] = {90, 89, 70}; // Priority for each thread

    pthread_t threads[NUM_THREADS];
    std::array<void *(*)(void *), NUM_THREADS> thread_funcs = {slaveReadWriteThread, controlThread,
                                                                calculateDynamicsThread};

    for (int i = 0; i < NUM_THREADS; ++i) {
    int ret = spawn_rt_thread(&threads[i], CPU[i], PRIORITY[i], thread_funcs[i], nullptr);
    if (ret != 0) {
        std::fprintf(stderr, "Error: spawn_rt_thread failed: %d\n", ret);
        return 1; // 프로그램 즉시 종료
    }
    }
    // // create threads with distinct functions and pin to cores
    // pthread_create(&threads[0], nullptr, threadA, nullptr);
    // rt_func_->pin_to_cpu(threads[0], 0);  
    // pthread_create(&threads[1], nullptr, threadB, nullptr);
    // rt_func_->pin_to_cpu(threads[1], 1);
    // pthread_create(&threads[2], nullptr, threadC, nullptr);
    // rt_func_->pin_to_cpu(threads[2], 2);

    // GUI 
    // w_Main.show(); ############################
    // int ret = a.exec();  // ############################


    
    // Thread 종료
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], nullptr);
    }

    // return ret;
}
