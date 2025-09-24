#include "Trajectory.hpp"

struct Trajectory::Impl{

    Vector2d pos_ref = Vector2d::Zero();
    Vector2d error = Vector2d::Zero();

    
    Impl()
    {
        
    }
     Vector2d stance_traj(double t)
    {
        double  f = 0;
        pos_ref[0] = 0;
        pos_ref[1] = -0.45 ;// - 0.05 * sin(2 * M_PI * f * t);

        return pos_ref;
    }
    
    Vector2d Landing_pos(double z_vel)
    {
        double qm;
        double qb;
        double q_max = 20;
        double L = 0.25;
        double vz_max;

        vz_max = -z_vel;

        qb = acos(-sqrt(2)/2 * vz_max/(q_max * L));
        qm = acos(-cos(qb));

        pos_ref[0] = -L*cos(qm) - L*cos(qb);
        pos_ref[1] = -L*sin(qm) - L*sin(qb);
        
        //! ROM Avoidance
        if(abs(pos_ref[1]) > 0.48)
        {
            pos_ref[1] = -0.48;
        }
        
        return pos_ref;
    }
};



Vector2d Trajectory::get_trajectory(double t, double z_vel, int Traj_mode)
{
    if(Traj_mode == 0)
    {
        return pimpl_->Landing_pos(z_vel);
    }
    else if(Traj_mode == 1)
    {
        return pimpl_->stance_traj(t);   
    }
    else
    {
        return Vector2d::Zero();
    }
    
}


Trajectory::Trajectory()
:pimpl_(std::make_unique<Impl>())
{
    
}

Trajectory::~Trajectory() = default;
