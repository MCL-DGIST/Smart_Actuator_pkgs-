#include "Kinematics.hpp"
#include "Actuator.hpp"
#include "Trajectory.hpp"

#include <cmath>        
#include <utility>      
#include <Eigen/Core>
#include "inter_process_state_manager.hpp"


using namespace Eigen;


struct Kinematics::Impl
{

    std::shared_ptr<Actuator> Act_;
    std::shared_ptr<Trajectory> Traj_;
    
    double L = 0.25;
    Vector2d q = Vector2d::Zero();
    
    Vector2d pos_ref = Vector2d::Zero();
    
    Vector2d pos = Vector2d::Zero();
    Vector2d vel = Vector2d::Zero();
    
    Vector2d pos_err = Vector2d::Zero();
    Vector2d pos_err_old = Vector2d::Zero();
    Vector2d vel_err = Vector2d::Zero();
    
    Vector2d j_pos_err = Vector2d::Zero();
    Vector2d j_pos_err_old = Vector2d::Zero();
    
    Matrix2d jacb = Matrix2d::Zero();


    Impl(std::shared_ptr<Actuator> A_,
        std::shared_ptr<Trajectory> T_
    )
    :Act_(std::move(A_)), 
     Traj_(std::move(T_))
    {

    }




    
    void Cal_kinematics(double t){

        q[0] = Act_->get_joint_angle(0);
        q[1] = Act_->get_joint_angle(1);

        jacb << L*sin(q[0]), L*sin(q[1]),
                -L*cos(q[0]), -L*cos(q[1]);

        pos[0] = - L*cos(q[0]) - L*cos(q[1]); //x
        pos[1] = - L*sin(q[0]) - L*sin(q[1]); //z

        
        pos_ref = Traj_->swing_traj(t);
        
        pos_err_old = pos_err;     
        pos_err = pos_ref - pos;
        
    }





};


Kinematics::Kinematics(std::shared_ptr<Actuator> A_, std::shared_ptr<Trajectory> T_)
:pimpl_(std::make_unique<Impl>(std::move(A_), std::move(T_)))                    
    {

    }

Kinematics::~Kinematics() = default;



void Kinematics::Cal_kinematics(double t) {
    pimpl_->Cal_kinematics(t);
}

Vector2d Kinematics::get_pos() {
    return pimpl_->pos;
}

Matrix2d Kinematics::get_jacobian() {
    return pimpl_->jacb;
}

Vector2d Kinematics::get_pos_err() {
    return pimpl_->pos_err;
};

Vector2d Kinematics::get_pos_err_old() {
    return pimpl_->pos_err_old;
};

