#include "Sensordata.hpp"
#include "globals.hpp"


struct Sensordata::Impl
{
    double z_vel = 0;
    double Fz = 0;
    Impl()
    {

    }

    double get_z_vel()
    {
        return z_vel;
    }
};


Sensordata::Sensordata()
:pimpl_(std::make_unique<Impl>())
{

}

Sensordata::~Sensordata()
{

}

void Sensordata::get_sensordata(const mjModel* m, mjData* d)
{
    pimpl_->z_vel = d->sensordata[2]; 
    pimpl_->Fz = d->sensordata[11];   
}

double Sensordata::get_z_vel(){return pimpl_->get_z_vel();}
double Sensordata::get_Fz(){return pimpl_->Fz;}
