
    #include "Integrate.hpp"
    #include "Trajectory.hpp"
    #include "Actuator.hpp"
    #include "Controller.hpp"
    #include "Kinematics.hpp"
    #include "Sensordata.hpp"

    struct Integrate::Impl
    {
        int Ctrl_mode = 1; // 0: Joint Space PID, 1: Task Space PID
        int Traj_mode = 0; // 0: Flight, 1: Stand
        bool Admittance_on = true;
        
        std::shared_ptr<Actuator> Act_;
        std::shared_ptr<Trajectory> Traj_;
        std::shared_ptr<Controller> Ctrl_;
        std::shared_ptr<Kinematics> Kin_;
        std::shared_ptr<Sensordata> Sen_;

        double t = 0;

        Vector2d Ctrl_input = Vector2d::Zero();
        Matrix2d jacb = Matrix2d::Identity();
        Vector2d pos_ref = Vector2d::Zero();

        Impl(std::shared_ptr<Actuator> A_,
            std::shared_ptr<Trajectory> T_,
            std::shared_ptr<Controller> C_,
            std::shared_ptr<Kinematics> K_,
            std::shared_ptr<Sensordata> S_

        )
        :Act_(std::move(A_)),
        Traj_(std::move(T_)),
        Ctrl_(std::move(C_)),
        Kin_(std::move(K_)),    
        Sen_(std::move(S_))    

        {
            
        }


        void Cal_Kinematics(double t)
        {
            
            double dz = 0;
            dz = Ctrl_->Admittance(1000,500,100,Sen_->get_Fz());
            Kin_->get_adm_dz(dz);

            pos_ref = Traj_->get_trajectory(t, Sen_->get_z_vel(), Traj_mode);
            
            Kin_->Cal_kinematics(t, pos_ref);
            jacb = Kin_->get_jacobian();

        }


        void Ctrl()
        {
            //! PID CONTROLLER
                if(Ctrl_mode == 0)
                {//* Joint Space PID
                    Ctrl_input = Ctrl_->j_PID(Kin_->get_j_pos_err(), Kin_->get_j_pos_err_old());   
                }
                else
                {//* Task Space PID
                    Ctrl_input = jacb.transpose() * Ctrl_->PID(Kin_->get_pos_err(), Kin_->get_pos_err_old());   
                }
                
                    Ctrl_input[0] = Ctrl_input[0] + Ctrl_input[1]; 
            
                
                    // cout << Ctrl_input[0] << "  " << Ctrl_input[1] << endl;
        }

        Logging_data make_logging_data()
        {
            Integrate::Logging_data log;

            log.t = t;
            log.pos     = Kin_->get_pos();
            log.pos_ref = pos_ref; 
            log.vel     = Kin_->get_vel();              // vel getter가 없으면 임시로 Zero
            log.vel_ref = Vector2d::Zero();              // vel_ref도 동일
            log.z_vel = Sen_->get_z_vel();

            return log;
        }

    };

    Integrate::Integrate(std::shared_ptr<Actuator> A_,
                        std::shared_ptr<Trajectory> T_,
                        std::shared_ptr<Controller> C_,
                        std::shared_ptr<Kinematics> K_,
                        std::shared_ptr<Sensordata> S_)
    :pimpl_(std::make_unique<Impl>(std::move(A_), 
                                std::move(T_),
                                std::move(C_),
                                std::move(K_),
                                std::move(S_))
                                )
                                
    {

    }

    Integrate::~Integrate() = default;


    void Integrate::Ctrl()
    {
        pimpl_->Ctrl();
    }





    /// Calculate Kinematics at time t
    /// This function is used in the control loop to get the current state of the robot
    /// and the desired state given by the trajectory.
    void Integrate::Cal_Kinematics(double t)
    {
        pimpl_->Cal_Kinematics(t);
    }


//* Getter Function 
    void Integrate::get_state(double time)
    {
        pimpl_->t = time;

        

    }
    Vector2d Integrate::get_joint_input()
    {
        return pimpl_->Ctrl_input;
    }

    Integrate::Logging_data Integrate::get_logging_data() {
    return pimpl_->make_logging_data();
    }