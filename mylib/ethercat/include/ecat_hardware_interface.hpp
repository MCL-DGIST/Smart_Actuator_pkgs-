#ifndef ETHERCAT_HELPER_H_
#define ETHERCAT_HELPER_H_

/* Dependencies */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "ethercat.h"  // * SOEM EtherCAT Master
#pragma GCC diagnostic pop

/* Custom libraries */
#include "ecat_data.hpp"

/* Define number of each slave */
#define TWITTER_1   1

/* Define name of each slave in order to specify slave */
//Able to check this information through 'slaveinfo test program'
#define TWITTER_1_NAME  "ModuleSlotsDrive" //Serial numbers may differ

//Sync Manage
#define SM2_RXPDO_ASSIGN    0x1C12 //Memory location where mapped PDOs are saved
#define SM3_TXPDO_ASSIGN    0x1C13


///* Definition of PDO mapping (Rx, Tx) */
#ifdef SLAVE_GTWI
/// Gold
//Define RxPDO mapping
//See the predefined PDO
//Locates by order that is written
typedef struct PACKED
{
    //int8_t = int8_t
    //Entry address: 0x1600
    int32_t RXPDO_TARGET_POSITION_DATA;   //subindex: 0x01 //0x607A:0x00(20)
    uint32_t RXPDO_DIGITAL_OUTPUTS_DATA;  //subindex: 0x02
    uint16_t RXPDO_CONTROLWORD_DATA;  //subindex: 0x03
    //Entry address: 0x1605
    int32_t RXPDO_TARGET_POSITION_DATA_0; //_0: same name with the first one //subindex: 0x01
    int32_t RXPDO_TARGET_VELOCITY_DATA;   //subindex: 0x02
    int16_t RXPDO_TARGET_TORQUE_DATA; //subindex: 0x03
    uint16_t RXPDO_MAXIMAL_TORQUE;    //subindex: 0x04
    uint16_t RXPDO_CONTROLWORD_DATA_0;    //subindex: 0x05
    int8_t RXPDO_MODE_OF_OPERATION_DATA;  //subindex: 0x06
    uint8_t RXPDO_RESERVED;   //subindex: 0x07 (Padding)
} output_GTWI_t;

//Define TxPDO mapping
typedef struct PACKED
{
    //Entry address: 0x1A02
    int32_t TXPDO_ACTUAL_POSITION_DATA;
    int16_t TXPDO_ACTUAL_TORQUE_DATA;
    uint16_t TXPDO_STATUSWORD_DATA;
    int8_t TXPDO_MODE_OF_OPERATION_DISPLAY_DATA;
    uint8_t TXPDO_RESERVED;
    uint32_t TXPDO_AUX_POSITION_ACTUAL_DATA;
    int32_t TXPDO_ACTUAL_VELOCITY_DATA; //0x606C
} input_GTWI_t;
#endif

#ifdef SLAVE_PTWI
///Platinum PDO
//Define RxPDO mapping
typedef struct PACKED
{
    uint16_t RXPDO_CONTROLWORD_DATA;  //0x6040
    int8_t RXPDO_MODE_OF_OPERATION_DATA; //0x6060
    int16_t RXPDO_TARGET_TORQUE_DATA; //0x6071
    uint32_t RXPDO_DIGITAL_OUTPUTS_DATA;  //0x60fe:0x01
} PtwiRxPDOMap;

//Define TxPDO mapping
typedef struct PACKED
{
    //uint8_t a;
    uint16_t TXPDO_STATUSWORD_DATA;     //0x6041
    int8_t TXPDO_MODE_OF_OPERATION_DISPLAY_DATA; //0x6061
    int32_t TXPDO_ACTUAL_POSITION_DATA; //0x6064
    int16_t TXPDO_ACTUAL_TORQUE_DATA; //0x6077
    uint32_t TXPDO_DC_LINK_CIRCUIT_VOLTAGE_DATA; //0x6079
    double TXPDO_ADDITIONAL_ACTUAL_POSITION_DATA; //0x2FE4:0x02
    int32_t TXPDO_ACTUAL_VELOCITY_DATA; //0x606C

} PtwiTxPDOMap;
#endif

#ifdef SLAVE_ANYBUS
//Define TxPDO mapping
typedef struct PACKED
{
    uint8_t FTOB[3];
} AnybusRxPDOMap;

typedef struct PACKED
{
    uint8_t TSIB[18];
#ifdef SLAVE_FT
    uint8_t FTIB[32];
#endif
} AnybusTxPDOMap;
#endif


/* Data addresses */
//Defined at main.cpp
extern uint16_t RXPDO_ADDR_GTWI[3];
extern uint16_t TXPDO_ADDR_GTWI[4];

extern uint16_t RXPDO_ADDR_PTWI[2];
extern uint16_t TXPDO_ADDR_PTWI[2];

extern uint16_t TXPDO_ADDR_TS[4];

/* Function pre-definitions */
int initEcatMaster(const char *ifname);
int ecat_PDO_Config_PTWI(uint16_t slave);
int ecat_PDO_Config_GTWI(uint16_t slave);
int ecat_PDO_Config_TS(uint16_t slave);

#endif // ECAT_FUNC_H
