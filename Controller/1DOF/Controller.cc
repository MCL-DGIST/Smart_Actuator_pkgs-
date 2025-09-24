#include "Controller.hpp"
#include <cmath>        // std::acos
#include <utility>      // std::move
#include <Eigen/Core>
#include "inter_process_state_manager.hpp"

using Eigen::Vector2d;

// 숨길 내부 구현체
struct Controller::Impl
{
    // Sampling time
    double Ts = 0.001;

    // Low-pass filter cutoff frequency
    double cutoff_freq = 100.0;
    double tau = 0.0;


    Eigen::Vector2d KP[4], KI[4], KD[4];

    double j_P_term = 0;
    double j_P_term_old  = 0;
    double j_I_term = 0;
    double j_I_term_old = 0;
    double j_D_term = 0;
    double j_D_term_old = 0;
    double j_PID_output = 0;
    double j_KP;
    double j_KI; 
    double j_KD; 



    
    Impl()
    {        // 안전한 pi 계산 (M_PI 비의존)
        tau = 1.0 / (2.0 * M_PI * cutoff_freq);
    }



    double joint_PID(const double& error, const double& error_old)
    {
 
        j_KP = 1.2;
        j_KI = 0;
        j_KD = 0.08;
        


        tau = 1.0/(2.0*M_PI*cutoff_freq);
        j_P_term = j_KP * (error);
        j_I_term  = j_KI * (Ts/2.0)*(error+error_old) + j_I_term_old;
        j_D_term  = 2.0*j_KD * (1.0/(2.0*tau+Ts))*(error-error_old)
                          -((Ts-2.0*tau)/(2.0*tau+Ts)) * j_D_term_old;
        
        j_PID_output = j_P_term + j_I_term + j_D_term; 

        j_P_term_old = j_P_term;
        j_I_term_old = j_I_term;
        j_D_term_old = j_D_term;
        
        return j_PID_output;
    
        
    }


    

    void get_gain()
    {
        auto& shm = InterProcessStateManager::getInstance(); // 이미 생성된 싱글턴
        auto acc = shm.getSafeAccess();                      // lock 자동

            KP[0][0] = acc->gui_cmd.kp_x[0]; //x 
            KP[0][1] = acc->gui_cmd.kp_z[0]; //z 
            KI[0][0] = acc->gui_cmd.ki_x[0]; //x 
            KI[0][1] = acc->gui_cmd.ki_z[0]; //z 
            KD[0][0] = acc->gui_cmd.kd_x[0]; //x 
            KD[0][1] = acc->gui_cmd.kd_z[0]; //z 
           
            // j_KP[0] = acc->gui_cmd.kp_x[0];
            // j_KP[1] = acc->gui_cmd.kp_z[0];
            // j_KD[0] = acc->gui_cmd.kd_x[0];
            // j_KD[1] = acc->gui_cmd.kd_z[0];

            // std::cout << KP[0][0] << KP[0][1] << std::endl;
        
        // std::cout << acc->gui_cmd.kp_x[0] << std::endl;
        // acc->gui_cmd
    }
};



Controller::Controller()
    : pimpl_(std::make_unique<Impl>())
{}

Controller::~Controller() = default;



double Controller::joint_PID(const double& error, const double& error_old)
{
    // pimpl_->get_gain();
    return pimpl_->joint_PID(error, error_old);
}

void Controller::get_gain()
{
    pimpl_->get_gain();
}