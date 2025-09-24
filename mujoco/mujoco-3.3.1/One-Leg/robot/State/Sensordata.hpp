#include <iostream>
#include <vector>
#include <memory>
#include <mujoco/mujoco.h>

class Sensordata
{
public:
    Sensordata(/* args */);
    ~Sensordata();
    void get_sensordata(const mjModel* m, mjData* d);
    double get_z_vel();
    double get_Fz();
    
private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

};

