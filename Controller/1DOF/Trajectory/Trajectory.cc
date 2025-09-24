#include "Trajectory.hpp"


struct Trajectory::Impl{

    Vector2d pos_ref = Vector2d::Zero();
    Vector2d error = Vector2d::Zero();

    
    Impl()
    {
        
    }
 Vector2d swing_traj(double t)
    {
        double  f = 3;
        pos_ref[0] = 0;
        pos_ref[1] = -0.3536 + 0.05*sin(2*M_PI*f*t);

        return pos_ref;
    }
};


Vector2d Trajectory::swing_traj(double t)
{
    return pimpl_ ->swing_traj(t);
}



Trajectory::Trajectory()
:pimpl_(std::make_unique<Impl>())
{
    
}

Trajectory::~Trajectory() = default;
