#pragma once

#include <cstdio>
#include <memory>

#include "globals.hpp"

class Integrate;

class dataLogging {
public:
    dataLogging();
    ~dataLogging();

    void cal_value(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I_);
    void init_save_data_leg(FILE* fid);
    void save_data_leg(const mjModel* m, mjData* d, std::shared_ptr<Integrate> I_,
                       FILE* fid);
    void init_save_data_Body(FILE* fid);
    // void save_data_Body(const mjModel* m, mjData* d, Body &B, FILE* fid);
    void initiate();
    FILE* getFid() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    
};

