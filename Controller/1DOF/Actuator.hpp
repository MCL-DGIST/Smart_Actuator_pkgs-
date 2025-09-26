#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <cstdint>   // uint16_t, int8_t
#include <array>
#include <data_mutex.hpp>
#include "process_shared_data.hpp"
#include "ecat_data.hpp"
#include "ecat_hardware_interface.hpp"

/**
 * @brief Actuator Class
 * @param target_torque: Load torque @param current: Motor current
 * 
 */
class Actuator
{
public:

    explicit Actuator(int numofslaves);
    ~Actuator();


    bool BindPDO();           // ← 추가: OP 전환 후 1회 호출

    void Receive_DATA();
    void Send_DATA();
    double get_joint_angle(int num);
    
    const Actuator_DATA& data() const;

    void resetMotorOffset();
    void setPtwiPdoPointers(void *ptwi_tx_pdo, void *ptwi_rx_pdo);
    void setGtwiPdoPointers(void *gtwi_tx_pdo, void *gtwi_rx_pdo);
    void setAnybusPdoPointers(void *anybus_tx_pdo);

private:
    
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    #ifdef SLAVE_GTWI
        GtwiRxPDOMap *gtwi_rx_pdo_;                 // RxPDO mapping data (output to the slaves)
        GtwiTxPDOMap *gtwi_tx_pdo_;                 // TxPDO mapping data (input from the slaves)
        ElmoGtwiDriver *elmo_gtwi_driver_{nullptr}; // Pointer to ELMO Gold driver data
    #endif

    #ifdef SLAVE_PTWI
        PtwiRxPDOMap *ptwi_rx_pdo_;                 // RxPDO mapping data (output to the slaves)
        PtwiTxPDOMap *ptwi_tx_pdo_;                 // TxPDO mapping data (input from the slaves)
        ElmoPtwiDriver *elmo_ptwi_driver_{nullptr}; // Pointer to ELMO Platinum driver data
    #endif
    
    #ifdef SLAVE_ANYBUS
        AnybusTxPDOMap *anybus_tx_pdo_;                    // TxPDO mapping data (input from the slaves)
        AnybusCommunicator *anybus_communicator_{nullptr}; // Pointer to Anybus communicator data
    #endif
};

