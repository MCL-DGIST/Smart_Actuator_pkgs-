#include "dataLogging.hpp"
#include "globals.hpp"
#include "Integrate.hpp"

#include <cstdio>
#include <utility>
#include <Eigen/Core>

using Eigen::Vector2d;
using Eigen::VectorXd;

using namespace std;

struct dataLogging::Impl {
    FILE* fid = nullptr;

    int loop_index = 0;
    const int data_frequency = 100;

    const char* datapath = "../data/data.csv";

    Integrate::Logging_data data;
    
    double t = 0.0;

    Vector2d Leg_pos_ref;
    Vector2d Leg_pos;

    VectorXd x0   = VectorXd::Zero(12);
    VectorXd x_ref= VectorXd::Zero(12);


    void cal_value(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I_)
    {
        t = d->time;
        data =I_->get_logging_data();
        
    }
     
    void save_data_leg(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I_,
                                FILE* fid)
    {
                cal_value(m, d, I_);

                fprintf(fid, "%f, %f, %f, %f, %f, %f, %f, ", t, data.pos[0], data.pos[1], data.pos_ref[0], data.pos_ref[1], data.vel[0], data.vel[1]);
                fprintf(fid, "%f", data.z_vel);
                fprintf(fid, "\n");
    }  
};

dataLogging::dataLogging()
: pimpl_(std::make_unique<Impl>()) {}

dataLogging::~dataLogging() {
    if (pimpl_->fid) {
        std::fclose(pimpl_->fid);
        pimpl_->fid = nullptr;
    }
}

void dataLogging::cal_value(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I_)
{
    pimpl_->cal_value(m, d, I_);
}

void dataLogging::init_save_data_leg(FILE* fid)
{
    if (fid) {
        fprintf(fid, "t, pos_x, pos_z, pos_x_ref, pos_z_ref, vel_x, vel_z, ");
        fprintf(fid, "z_vel");
        fprintf(fid, "\n");
    }
}

void dataLogging::save_data_leg(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I,
                                FILE* fid)
{
    pimpl_->save_data_leg(m, d, I, fid);
}

void dataLogging::initiate()
{
    pimpl_->fid = std::fopen(pimpl_->datapath, "w");
    init_save_data_leg(pimpl_->fid);
}

FILE* dataLogging::getFid() const {
    return pimpl_->fid;
}
