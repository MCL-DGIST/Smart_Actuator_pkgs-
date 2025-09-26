
    #include "Integrate.hpp"
    #include "Trajectory.hpp"
    #include "Actuator.hpp"
    #include "Controller.hpp"
    #include "Kinematics.hpp"
    #include "inter_process_state_manager.hpp"

    struct Integrate::Impl
    {
        Control_DATA c;
        
        std::shared_ptr<Actuator> Act_;
        std::shared_ptr<Trajectory> Traj_;
        std::shared_ptr<Controller> Ctrl_;

        double t = 0;

        double Ctrl_input = 0;

        Impl(std::shared_ptr<Actuator> A_,
            std::shared_ptr<Trajectory> T_,
            std::shared_ptr<Controller> C_

        )
        :Act_(std::move(A_)),
        Traj_(std::move(T_)),
        Ctrl_(std::move(C_))   

        {
            
        }

        void Send_to_Gui()
        {            

         
        }


        void Ctrl()
        {
            //! PID CONTROLLER
            // Ctrl_input = Ctrl_->joint_PID()            
            // // cout << "Ctrl_input:  " << Ctrl_input[0] << "  " << Ctrl_input[1] << endl;  
            
        }


    };

    Integrate::Integrate(std::shared_ptr<Actuator> A_,
                        std::shared_ptr<Trajectory> T_,
                        std::shared_ptr<Controller> C_)
    :pimpl_(std::make_unique<Impl>(std::move(A_), 
                                std::move(T_),
                                std::move(C_)))
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



//* Getter Function 
    void Integrate::get_state(double time)
    {
        pimpl_->t = time;

            

    }
    double Integrate::get_joint_input()
    {
        return pimpl_->Ctrl_input;
    }

    const Control_DATA& Integrate::data() const 
    {
        pimpl_->Send_to_Gui();
        return pimpl_->c;
    }