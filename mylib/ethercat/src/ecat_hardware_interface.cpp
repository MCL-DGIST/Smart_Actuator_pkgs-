#include "ecat_hardware_interface.hpp"

#include <cstring>
#include <iostream>
#include <stdio.h>

using namespace std;

// Gold Twitter
// Mensioned in ecat_func.h as extern
//{No. of entries to be mapped, Entry addr., Entry addr., ...}
uint16_t RXPDO_ADDR_GTWI[3] = {2, 0x1600, 0x1605}; //
// uint16_t TXPDO_ADDR_GTWI[5] = {4, 0x1A02, 0x1A03, 0x1A18, 0x1A1D};
// uint16_t TXPDO_ADDR_GTWI[6] = {5, 0x1A02, 0x1A03, 0x1A18, 0x1A1D, 0x1A1E};
uint16_t TXPDO_ADDR_GTWI[4] = {3, 0x1A02, 0x1A1E, 0x1A11};

// Platinum Twitter
uint16_t RXPDO_ADDR_PTWI[2] = {1, 0x1600};
uint16_t TXPDO_ADDR_PTWI[2] = {1, 0x1A00};

// TS
uint16_t TXPDO_ADDR_TS[4] = {3, 0x1A01, 0x1A02, 0x1A03};

/**
 * @brief Initialize EtherCAT Master
 * - Search for slaves on network
 * - Check number of slaves and their names
 * @param ifname : Device inteface name (i.e. EtherNET port name e.g. eth0)
 * @return 1 (success) / 0 (error)
 */

/* Functions */
// Used for EtherCAT initialization
// If there was any error, it returns 0.
int initEcatMaster(const char *ifname) // ifname: Interface name (name of ethernet port)
{
  int i, oloop, iloop, k, wkc_count;

  printf("Initializing EtherCAT Master...\n");
  // printf("before ec_init\n");
  // fflush(stdout);
  // int ok = ec_init(ifname);
  // printf("after ec_init=%d (errno=%d)\n", ok, errno);
  // fflush(stdout);

  // printf("before ec_config_init\n");
  // fflush(stdout);
  // int ok2 = ec_config_init(FALSE);
  // printf("after ec_config_init=%d (errno=%d), ec_slavecount=%d\n", ok2, errno, ec_slavecount);
  // fflush(stdout);

  if (ec_init(ifname) > 0) // when ec_init is succeeded
  {
    printf("ec_init on %s succeeded.\n", ifname);

    if (ec_config_init(FALSE) > 0) // Returns workcounter of slave discover datagram
    {
      printf("%d slaves found and configured.\n",
             ec_slavecount); // ec_slavecount: number of slaves found on the network

      if (ec_slavecount < SLAVES_NUM) // Detected slaves are less than actual number of slaves
      {
        cout << "Error: Detected number of ECAT slaves are less than Actual slaves" << endl;
        return 0;
      }
      //            if(strcmp(ec_slave[TWITTER_1].name, TWITTER_1_NAME)) //Compare name of slave.
      //            ec_slave[0] reserves for master.
      //            {
      //                char *nametmp; //Pointer for string. eg) char *s1 = "Hello";
      // printf("%s\n",
      //                s1);=Hello nametmp = ec_slave[TWITTER_1].name; printf("Name of slave is
      // %s,
      //                not TWITTER.\n", nametmp); return 0;
      //            }
    }
    else { return 0; }
  }
  else {
    cout << "ec_init failed!\n";
    return 0;
  }
  return 1;
}
#ifdef SLAVE_GTWI // Since SLAVE_GTWI is defined at ecat_func.h, codes in here are activated
int ecat_PDO_Config_GTWI(uint16 slave) {
  int retval_SM2 = 0;
  int retval_SM3 = 0;

  // ec_SDOwrite(slave number, Index to write, Subindex to write(must be 0 or 1, if CA used),
  // CA(T:Complete access all subindexes written//F:single subindex), size in bytes of parameter
  // buffer, pointer to parameter buffer, Timeout in microsec(standard is EC_TIMEOUTRXM)) SDOwrite
  // is kind of write in Supervisor mode
  retval_SM2 += ec_SDOwrite(slave, SM2_RXPDO_ASSIGN, 0x00, TRUE, sizeof(RXPDO_ADDR_GTWI),
                            &RXPDO_ADDR_GTWI, EC_TIMEOUTSAFE); // EC_TIMEOUTSAFE: 20000 us
  retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_GTWI),
                            &TXPDO_ADDR_GTWI, EC_TIMEOUTSAFE);

  return 1;
}
#endif

#ifdef SLAVE_PTWI
int ecat_PDO_Config_PTWI(uint16 slave) {
  int retval_SM2 = 0;
  int retval_SM3 = 0;

#ifdef SLAVE_PTWI
  int8 b = 0;
  uint16 w, ind;
  uint32 dw;

  // Motor setting
  //         uint16 max_torque = 4050;
  uint16 max_torque = 2000;
  uint32 Rated_torque = 12120;
  ec_SDOwrite(slave, 0x6072, 0, FALSE, sizeof(max_torque), &max_torque, EC_TIMEOUTRXM);
  ec_SDOwrite(slave, 0x6073, 0, FALSE, sizeof(max_torque), &max_torque, EC_TIMEOUTRXM);
  // rpdo----------------
  // 0x1600
  //
  ind = 0;
  ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  dw = 0x60400010;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x60600008;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x60710010;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x60fe0120; // 60feh( D out )
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  b = ind; // 0x00 => Number of subindexes
  ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

  // SM2
  retval_SM2 += ec_SDOwrite(slave, SM2_RXPDO_ASSIGN, 0x00, TRUE, sizeof(RXPDO_ADDR_PTWI),
                            &RXPDO_ADDR_PTWI, EC_TIMEOUTSAFE); // EC_TIMEOUTSAFE: 20000 us

  // tpdo----------------

  // 1a00
  ind = 0;
  b = 0;
  ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  // dw = htoel_(0x60410010);//6041h( Status word )
  dw = 0x60410010;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  // dw = htoel_(0x60610008);//6061h( Mode of Operation Disp )
  dw = 0x60610008;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  // dw = htoel_(0x60640020);//6064h ( Position feedback )
  dw = 0x60640020;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  // dw = htoel_(0x60770010);//6077h ( Torque feedback )
  dw = 0x60770010;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x60790020;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x2fe40240;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x606c0020;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  b = ind;
  ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

  // 1c13.0
  // SM3
  b = 0;
  ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  w = 0x1a00;
  ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(w), &w, EC_TIMEOUTRXM);
  b = 1;
  ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_PTWI),
                            &TXPDO_ADDR_PTWI, EC_TIMEOUTSAFE);

#endif
  return 1;
}
#endif

#ifdef SLAVE_ANYBUS
int ecat_PDO_Config_TS(uint16 slave) {
  int retval_SM2 = 0;
  int retval_SM3 = 0;

  int8 b = 0;
  uint16 w, ind;
  uint32 dw;
  // rpdo---------------
#ifdef SLAVE_FT
  ind = 0;
  b = 0;
  ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  dw = 0x21000108;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x21000208;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x21000308;
  ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  b = ind;
  ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
#endif
  // tpdo----------------

  // 1a01
  ind = 0;
  b = 0;
  ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  dw = 0x20000108;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000208;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000308;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000408;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

  dw = 0x20000508;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000608;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000708;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000808;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000908;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000a08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000b08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000c08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000d08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000e08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20000f08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001008;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001108;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001208;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

#ifdef SLAVE_FT
  dw = 0x20001308;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001408;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001508;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001608;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001708;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001808;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001908;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001a08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

  dw = 0x20001b08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001c08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001d08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001e08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20001f08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002008;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002108;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002208;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

  dw = 0x20002308;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002408;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002508;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002608;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002708;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002808;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002908;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002a08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

  dw = 0x20002b08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002c08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002d08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002e08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20002f08;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20003008;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20003108;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
  dw = 0x20003208;
  ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
#endif

  b = ind;
  ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

  // 1c13.0
  // SM3
  b = 0;
  ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  w = 0x1a00;
  ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(w), &w, EC_TIMEOUTRXM);
  b = 1;
  ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
  retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_TS),
                            &TXPDO_ADDR_TS, EC_TIMEOUTSAFE);

  return 1;
}
#endif
