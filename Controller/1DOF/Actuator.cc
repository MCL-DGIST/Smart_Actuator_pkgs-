#include "Actuator.hpp"
#include "ecat_func.hpp"
#include <ethercat.h>
#include <cmath>
#include "inter_process_state_manager.hpp"
#include "filter.hpp"

//* Somanet은 gear비가 고려된 load torque를 입력으로 받고, 또 출력함

struct Actuator::Impl{

    std::unique_ptr<filter> F; 

    int numofslaves;
    double torque_constant = 0.241679;
    double GearRatio = 100.0;
    double controlword[NUMOFSLAVES];

    // Circulo 9 (SOMANET) 구조체 포인터 정의
    std::vector<output_SOMANET_t*> out_somanet;  // RxPDO (우리가 슬레이브로 보내는 데이터)
    std::vector<input_SOMANET_t*>  in_somanet;    // TxPDO (슬레이브로부터 받는 데이터)


    Actuator_DATA a;
    // GUI_Command gui_cmd;
    
    int sigRTthreadKill = 0;
    uint16_t slave_state[NUMOFSLAVES];

    Impl(int n) 
    : numofslaves(n),
    out_somanet(n, nullptr), in_somanet(n, nullptr), F(std::make_unique<filter>())
    {}
    
    bool BindPDO() {
        // SOEM 슬레이브 인덱스는 1..ec_slavecount
        if (numofslaves < 1 || numofslaves > ec_slavecount) return false;

        // Set pointer size
        in_somanet.resize(numofslaves, nullptr);
        out_somanet.resize(numofslaves, nullptr);

        a.numofslaves = static_cast<uint32_t>(numofslaves);

        for (int i = 0; i < numofslaves; ++i) {
            int si = i + 1; // 1-based → 0-based 매핑
            in_somanet[i]  = reinterpret_cast<input_SOMANET_t*>(ec_slave[si].inputs);
            out_somanet[i] = reinterpret_cast<output_SOMANET_t*>(ec_slave[si].outputs);
            if (!in_somanet[i] || !out_somanet[i]) {
                std::fprintf(stderr, "BindPDO: null PDO at slave %d (in=%p, out=%p)\n",
                             si, (void*)in_somanet[i], (void*)out_somanet[i]);
                return false;
            }
        }
        return true;
    }

    void Receive_DATA()
    {
        wkc = ec_receive_processdata(EC_TIMEOUTRET); //returns WKC

        if(!sigRTthreadKill) //perform the task untill the kill signal is reached
        {
            // ec_send_processdata(); //PDO sending
            // wkc = ec_receive_processdata(EC_TIMEOUTRET); //returns WKC

             for (int i = 1; i <= numofslaves; i++)
                slave_state[i] = ec_slave[i].state;
            

            if(expectedWKC>wkc)
            {
                sigRTthreadKill = 1; //Kill the entire of the RT thread
            }


        for (int i = 0; i < numofslaves; ++i) {
            auto* in = in_somanet[i];           // in_somanet이 vector/배열이어야 함
            if (!in) continue;

            // 값 채우기 (벡터 크기는 BindPDO에서 numofslaves로 resize 되어 있어야 함)
            a.pos_raw[i]       = static_cast<double>(in->position_actual_value);
            a.vel_raw[i]       = static_cast<double>(in->velocity_actual_value);
            a.torque_raw[i]    = static_cast<double>(in->torque_actual_value);
            a.statusword[i]    = in->statusword;                   // uint16_t 벡터
            a.modeofOP_disp[i] = in->mode_of_operation_display;    // int8_t 벡터

            
            a.position_raw_calc[i]   = ((double)a.pos_raw[i]) * (2*M_PI / 1048576.0 / 9.0);
            a.position_refined[i] = a.position_raw_calc[i]-a.motor_offset[i];
            a.joint_angle[i] = a.position_refined[i];
            a.current[i] = a.torque_raw[i] * (20.0/1000.0) / (torque_constant * GearRatio);

        }

        //HIP MOTOR
        a.joint_angle[0] =  - a.position_refined[0] + M_PI/4;
        //KNEE MOTOR
        a.joint_angle[1] = a.position_refined[1] + M_PI*3/4;
        
        // a.joint_vel[0] = F->lowpassfilter()
            // 단위: 정격토크/1000 ex) 100 -> 정격토크*0.1
            // JD10 정격토크: 20Nm
            // Ain1_raw     = in_somanet->analog_input[0];  // analog_input[0~3]
            // Din_raw      = in_somanet->digital_inputs;
        }
    
    }

    void Send_DATA()
    {
        //* GUI command from Shared Memory *//
            {
            auto& shm = InterProcessStateManager::getInstance(); // 이미 생성된 싱글턴
            auto acc = shm.getSafeAccess();                      // lock 자동
            
                for(int i = 0; i < NUMOFSLAVES; i++)
                {
                    controlword[i] = acc->gui_cmd.controlword[i];
                }
                
            } 

            
        for(int i=0;i<NUMOFSLAVES;i++)
        {
            if(d.target_torque[i] > 8 || d.target_torque[i] < -8 )
            {
                if(controlword[i] = 128)
                    std::cout << i << " Motor torque is out of range" << std::endl;
                
                d.target_torque[i] = 0;
                controlword[i] = 0;
            }
            // out_somanet->target_position = (int32_t)((target_position[i] + motor_offset[i]) / (360.0 / 1048576.0 / 9.0));
            // out_somanet->target_velocity = (int32_t)(target_speed[i] / (360.0 / 1048576.0 / 9.0));
            if(i == 1) // KNEE MOTOR
            {
                out_somanet[i]->target_torque   = (d.target_torque[i]) * (1000.0 / 20.0); //단위: 정격토크/1000 ex) 1000 -> 정격토크
            }
            else // HIP MOTOR
            {
                out_somanet[i]->target_torque   = (-d.target_torque[i]) * (1000.0 / 20.0);
            }
            out_somanet[i]->mode_of_operation = 10;
            out_somanet[i]->controlword     = controlword[i];
            
            
        }

        // std::cout << "Ctrl input: " << -d.target_torque[0] << "  " <<  -d.target_torque[1] << std::endl;
    }




    void resetMotorOffset() {
        for (int i = 0; i < numofslaves; ++i) {
            // a.motor_offset을 현재 측정된 raw 각도 값으로 지정
            a.motor_offset[i] = a.position_raw_calc[i];
        }
    }



};


Actuator::Actuator(int n)
    :pimpl_(std::make_unique<Impl>(n))
{}

Actuator::~Actuator() = default;

void Actuator::Receive_DATA()
{
    pimpl_->Receive_DATA();
    // std::cout << pimpl_->numofslaves << "   " << pimpl_->pos_load[0] << "  " << pimpl_->pos_load[1] << std::endl;
}
void Actuator::Send_DATA()
{
    pimpl_->Send_DATA();
}

double Actuator::get_joint_angle(int num)
{
    return pimpl_->a.joint_angle[num];
    // std::cout << pimpl_->a.pos_load[0] << std::endl;
}
bool Actuator::BindPDO() { return pimpl_->BindPDO(); }

const Actuator_DATA& Actuator::data() const {return pimpl_->a;}



void Actuator::resetMotorOffset() {
    pimpl_->resetMotorOffset();
}



// double Actuator::get_pos_load(int ACT_NUM){return pimpl_->pos_load[ACT_NUM];}
// double Actuator::get_vel_load(int ACT_NUM){return pimpl_->vel_load[ACT_NUM];}

