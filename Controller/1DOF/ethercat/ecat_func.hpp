#pragma once

// 표준 및 시스템 헤더
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/timerfd.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <error.h>
#include <errno.h>
#include <sys/mman.h>
#include <inttypes.h>

// 데이터 보호용 뮤텍스 헤더
#include "data_mutex.hpp"

// SOEM EtherCAT 라이브러리 헤더
#include "ethercat.h"

// ---------------------- 프로젝트/슬레이브 정의 -------------------------

#define NUMOF_SLAVES    1           // 슬레이브 수
#define ELMO_1       1           // 첫 번째 슬레이브 인덱스




#define ELMO_1_NAME  "ModuleSlotsDrive"   // slaveinfo 결과에 기반한 슬레이브 이름

// PDO 매핑을 위한 Sync Manager (SM) 인덱스
#define SM2_RXPDO_ASSIGN    0x1C12  // RxPDO가 저장된 주소
#define SM3_TXPDO_ASSIGN    0x1C13  // TxPDO가 저장된 주소


// ---------------------- RxPDO: Master → Slave -------------------------
// Circulo에 명시된 순서와 데이터 타입 기준
typedef struct PACKED
{
    uint16_t controlword;               // 제어 상태 전이 명령 (0x6040)
    int8_t   mode_of_operation;         // 제어 모드 설정 (0x6060)
    int16_t  target_torque;             // 목표 토크 (0x6071)
    int32_t  target_position;           // 목표 위치 (0x607A)
    int32_t  target_velocity;           // 목표 속도 (0x60FF)
    int16_t  torque_offset;             // 토크 오프셋 (0x60B2)
    uint32_t tuning_command;            // 커맨드 (튜닝용?) (0x2701)
    uint32_t physical_outputs;          // 물리 출력 (0x60FE:01)
    uint32_t bit_mask;                  // 출력 마스크 (0x60FE:02)
    uint32_t user_mosi;                 // 사용자 정의 MOSI (0x2703)
    int32_t  velocity_offset;           // 속도 오프셋 (0x60B1)
} output_ELMO_t;


// ---------------------- TxPDO: Slave → Master -------------------------
// Circulo에서 슬레이브가 마스터에게 보내는 상태 정보
typedef struct PACKED
{
    uint16_t statusword;                // 현재 상태 (0x6041)
    int8_t   mode_of_operation_display; // 현재 동작 모드 (0x6061)
    int32_t  position_actual_value;     // 현재 위치 (0x6064)
    int32_t  velocity_actual_value;     // 현재 속도 (0x606C)
    int16_t  torque_actual_value;       // 실제 토크 (0x6077)
    uint16_t analog_input[4];           // 아날로그 입력 1~4 (0x2401~2404)
    uint32_t tuning_status;             // 튜닝 상태 정보 (0x2702)
    uint32_t digital_inputs;            // 디지털 입력 (0x60FD)
    uint32_t user_miso;                 // 사용자 정의 MISO (0x2704)
    uint32_t timestamp;                 // 타임스탬프 (0x20F0)
    int32_t  position_demand_internal;  // 내부 위치 목표값 (0x60FC)
    int32_t  velocity_demand_value;     // 속도 명령 (0x606B)
    int16_t  torque_demand;             // 토크 명령 (0x6074)
} input_ELMO_t;


// ---------------------- 외부 정의된 PDO 주소 배열 -------------------------
// main.cpp에서 이 주소 배열을 초기화함
extern uint16_t RXPDO_ADDR_ELMO[2];  // RxPDO 주소들
extern uint16_t TXPDO_ADDR_ELMO[2];  // TxPDO 주소들

// ---------------------- EtherCAT 초기화 함수 정의 -------------------------
int ecat_init(const char *ifname);        // EtherCAT 초기화 함수
int ecat_PDO_Config(uint16 slave);       // PDO 매핑 설정 함수


// main.cc에서 정의됨
extern const char* IFNAME;      // e.g., "enp2s0"
extern char IOmap[4096];        // PDO 매핑 버퍼

// main.cc에서 정의됨 (PDO 포인터)
extern output_ELMO_t* out_ELMO[NUMOF_SLAVES];
extern input_ELMO_t*  in_ELMO [NUMOF_SLAVES];

// main.cc에서 정의됨 (WKC 공유)
extern int expectedWKC;
extern volatile int wkc;

int soem_init();  // IFNAME 매크로를 사용해 초기화(아래 .cc 참고)
// uint16_t read_error_register_with_sdo(int slave_index);
