#include "ecat_func.hpp"
#include <iostream>

boolean needlf;
boolean inOP;
uint8 currentgroup = 0;

// RxPDO 매핑: 0x1600 하나
    uint16_t RXPDO_ADDR_ELMO[2] = {
        1, 0x1600  // entry 개수 1개, 주소는 0x1600
    };

// TxPDO 매핑: 0x1A00 하나
    uint16_t TXPDO_ADDR_ELMO[2] = {
        1, 0x1A00  // entry 개수 1개, 주소는 0x1A00
    };


/* EtherCAT 초기화 함수 */
int ecat_init(const char *ifname)
{
    needlf = FALSE;

    printf("Initializing EtherCAT Master...\n");

    if (ec_init(ifname) > 0)
    {
        printf("ec_init on %s succeeded.\n", ifname);

        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            if (ec_slavecount < NUMOF_SLAVES)
            {
                printf("Not enough slaves found.\n");
                return 0;
            }

            // 슬레이브 이름 확인
            if (strcmp(ec_slave[ELMO_1].name, ELMO_1_NAME))
            {
                printf("Name of slave is %s, not %s.\n", ec_slave[ELMO_1].name, ELMO_1_NAME);
                return 0;
            }
        }
        else
        {
            printf("ec_config_init failed.\n");
            return 0;
        }
    }
    else
    {
        printf("ec_init failed.\n");
        return 0;
    }

    return 1;
}

/* PDO 매핑 함수 */
int ecat_PDO_Config(uint16 slave)
{
    int retval_SM2 = 0;
    int retval_SM3 = 0;

    // RxPDO 매핑 설정 (0x1C12)
    retval_SM2 = ec_SDOwrite(slave, SM2_RXPDO_ASSIGN, 0x00, TRUE,
                             sizeof(RXPDO_ADDR_ELMO), &RXPDO_ADDR_ELMO, EC_TIMEOUTSAFE);

    // TxPDO 매핑 설정 (0x1C13)
    retval_SM3 = ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE,
                             sizeof(TXPDO_ADDR_ELMO), &TXPDO_ADDR_ELMO, EC_TIMEOUTSAFE);

    if (retval_SM2 <= 0 || retval_SM3 <= 0)
    {
        printf("PDO config failed: SM2=%d, SM3=%d\n", retval_SM2, retval_SM3);
        return 0;
    }

    return 1;
}

int soem_init()
{
    // 1) 마스터/슬레이브 검색
    if (!ecat_init(IFNAME)) {
        return 0;
    }

    // 2) 슬레이브 수 검증 및 PO->SO 설정 콜백 등록
    if (ec_slavecount != NUMOF_SLAVES) {
        printf("Slavecount(%d) != NUMOF_SLAVES(%d)\n", ec_slavecount, NUMOF_SLAVES);
        return 0;
    }

    for (int i = 1; i <= NUMOF_SLAVES; ++i) {
        ec_slave[i].PO2SOconfig = ecat_PDO_Config;
    }

    // 3) PDO 매핑 & DC 설정
    if (ec_config_map(&IOmap) <= 0) {
        printf("ec_config_map failed.\n");
        return 0;
    }

    ec_configdc(); // 실패해도 진행은 가능(DC 미지원 슬레이브가 있을 수도 있음)

    // // 4) 사용자 구조체 포인터 바인딩
    // for (int i = 0; i < NUMOF_SLAVES; ++i) {
    //     out_somanet[i] = (output_SOMANET_t*) ec_slave[i + 1].outputs;
    //     in_somanet[i]  = (input_SOMANET_t*)  ec_slave[i + 1].inputs;
    // }

    // 5) SAFE_OP까지 상태 전이 확인
    ec_statecheck(0, EC_STATE_SAFE_OP, 4 * EC_TIMEOUTSTATE);

    // 6) 예상 WKC 계산(디버깅 참고용)
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Expected WKC = %d\n", expectedWKC);

    // 7) OP 전환 준비(1회 PDO 왕복)
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    printf("First receive WKC = %d\n", wkc);

    // 8) 전 슬레이브에 OP 요청 & 확인
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    printf("All slaves requested to OP.\n");
    return 1;
}

// ---------------------- SDO로 Error Code 읽기 (옮겨온 함수) ----------------------
// uint16_t read_error_register_with_sdo(int slave_index)
// {
//     uint16_t error_code = 0;
//     int size = sizeof(error_code);
//     int ret = ec_SDOread(slave_index, 0x603F, 0x00, FALSE,
//                          &size, &error_code, EC_TIMEOUTSTATE);

//     if (ret > 0) {
//         printf("Slave %d: SDO read successful. Error Code = 0x%X\n",
//                slave_index, error_code);
//     } else {
//         printf("Slave %d: Failed to read Error Register. SDO read returned %d\n",
//                slave_index, ret);
//     }
//     return error_code;
// }