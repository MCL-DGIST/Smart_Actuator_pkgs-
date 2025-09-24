#include "Kinematics.hpp"
#include "Actuator.hpp"
#include "Trajectory.hpp"

#include <cmath>        
#include <utility>      
#include <Eigen/Core>



using namespace Eigen;


struct Kinematics::Impl
{

    std::shared_ptr<Actuator> Act_;
    std::shared_ptr<Trajectory> Traj_;
    
    double t = 0; 
    double L = 0.25;
    Vector2d q = Vector2d::Zero();
    
    Vector2d pos_ref = Vector2d::Zero();    
    Vector2d pos = Vector2d::Zero();
    Vector2d pos_err = Vector2d::Zero();
    Vector2d pos_err_old = Vector2d::Zero();
    
    Vector2d vel = Vector2d::Zero();
    Vector2d vel_err = Vector2d::Zero();
    
    Matrix2d jacb = Matrix2d::Zero();

    double z_vel = 0;


    //* Joint space Control
        Vector2d j_pos_ref = Vector2d::Zero();
        Vector2d j_pos_err = Vector2d::Zero();
        Vector2d j_pos_err_old = Vector2d::Zero();
    
    //* Manipulability 
        double q_max = 20; //rad/s
        double qb = 0;
        double qm = 0;
    
    //* Admittance Control
        double dz = 0;

        Impl(std::shared_ptr<Actuator> A_,
        std::shared_ptr<Trajectory> T_
    )
    :Act_(std::move(A_)), 
     Traj_(std::move(T_))
    {

    }




    
    void Cal_kinematics(double time, Vector2d pos_ref){

        t = time;
        
        q[0] = Act_->get_joint_angle(0);
        q[1] = Act_->get_joint_angle(1);

        jacb << L*sin(q[0]), L*sin(q[1]),
                -L*cos(q[0]), -L*cos(q[1]);

        pos[0] = - L*cos(q[0]) - L*cos(q[1]); //x
        pos[1] = - L*sin(q[0]) - L*sin(q[1]); //z
        
        vel = jacb * q;

        pos_err_old = pos_err;     
        pos_err = pos_ref - pos;


        
    }


    Vector2d get_j_pos_err()
    {
        
        j_pos_ref[0] = qm;
        j_pos_ref[1] = qb;
        double x_ref = 0;
        double z_ref = -0.4;
        
        j_pos_err_old = j_pos_err;
        j_pos_err = j_pos_ref - q;
        return j_pos_err;   
    }

    

};


Kinematics::Kinematics(std::shared_ptr<Actuator> A_, std::shared_ptr<Trajectory> T_)
:pimpl_(std::make_unique<Impl>(std::move(A_), std::move(T_)))                    
    {

    }

Kinematics::~Kinematics() = default;



void Kinematics::Cal_kinematics(double t, Vector2d pos_ref) {
    pimpl_->Cal_kinematics(t, pos_ref);
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

Vector2d Kinematics::get_j_pos_err() {
    return pimpl_->get_j_pos_err();
}

Vector2d Kinematics::get_j_pos_err_old() {
    return pimpl_->j_pos_err_old;
}

void Kinematics::get_adm_dz(double delta_z) {
    pimpl_->dz = delta_z;
}
Vector2d Kinematics::get_vel() {
    return pimpl_->vel;
}