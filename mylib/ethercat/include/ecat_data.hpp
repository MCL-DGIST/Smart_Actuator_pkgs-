/** @file elmo_data_types.hpp
 *  @brief Common data structures for ELMO controller system
 *  @author Jeongwoo Hong (jwhong1209@gmail.com)
 *  @note Shared data types for both multi-threading and multi-processing
 */

#ifndef ETHERCAT_DATA_H_
#define ETHERCAT_DATA_H_

#include <array>
#include <cstdint>

#define SLAVES_NUM 1      // Number of ELMO slaves
#define PTWI_SLAVES_NUM 1 // Number of ELMO Platinum slaves
#define FT_SENSOR_NUM   // Number of Force/Torque sensors
#define SLAVE_PTWI 1 
#define NON_SLAVE_TS

// #define SLAVE_ANYBUS  
// #define SLAVE_FT
// #define SLAVE_FT_NUM  // Number of Force/Torque sensors



/**
 * @brief EtherCAT communication status
 * Used for monitoring real-time communication health
 */
typedef struct ethercat_status {
  uint16_t slave_state[SLAVES_NUM + 1]; // FSM states for each slave
  uint16_t expected_wkc;                // Expected working counter
  int wkc;                              // Actual working counter
} EthercatStatus;

/**
 * @brief ELMO servo driver data structure
 * Contains all PDO data and configuration for one ELMO slave
 */
typedef struct elmo_ptwi_driver {
  /* RxPDO Data (Commands to slave) */
  uint16_t controlword;        // CANopen controlword
  int8_t mode_of_operation;    // CANopen mode of operation
  int16_t target_torque_rated; // Target torque [per thousand of rated torque]
  uint32_t digital_output;     // Digital outputs

  /* TxPDO Data (Feedback from slave) */
  uint16_t statusword;             // CANopen statusword
  int8_t mode_of_operation_disp;   // CANopen mode of operation display
  int32_t actual_position_cnt;     // Actual position [counts]
  int16_t actual_torque_rated;     // Actual torque [per thousand of rated torque]
  uint32_t dclink_circuit_voltage; // DC link voltage
  double additional_position_cnt;  // Actual position [counts]
  int32_t actual_velocity_cnt;     // Actual velocity [counts/s]
} ElmoPtwiDriver;

typedef struct elmo_gtwi_driver {
  double current_limit;     // Current limit [A]
  uint32_t velocity_factor; // TODO: Check this value using command FC[9] / FC[10]

  /* RxPDO Data (Commands to slave) */
  int32_t target_position_cnt; // Target position [counts] // * quadrature
                               // should be considered
  int32_t target_velocity_cnt; // Target velocity [counts/s] // * quadrature
                               // should be considered
  int16_t target_torque_rated; // Target torque [per thousand of rated torque]
  uint32_t digital_output;     // Digital outputs
  uint16_t controlword;        // CANopen controlword
  int8_t mode_of_operation;    // CANopen mode of operation
  // uint16_t maximal_torque;      // Max torque [per thousand of rated torque]

  /* TxPDO Data (Feedback from slave) */
  int32_t actual_position_cnt;     // Actual position [counts] // * quadrature
                                   // should be considered
  int32_t actual_velocity_cnt;     // Actual velocity [counts/s] // * quadrature
                                   // should be considered
  int16_t actual_torque_rated;     // Actual torque [per thousand of rated torque]
  uint16_t statusword;             // CANopen statusword
  int8_t mode_of_operation_disp;   // CANopen mode of operation display
  uint32_t digital_inputs;         // Digital inputs (TxPDO)
  uint32_t dclink_circuit_voltage; // DC link voltage
  int16_t analog_input;            // Analog input
} ElmoGtwiDriver;

typedef struct anybus_communicator {
  /* RxPDO Data (Commands to slave) */
  uint8_t ftob[3]; // Force sensor data

  /* TxPDO Data (Feedback from slave) */
  std::array<std::array<uint8_t, 2>, SLAVES_NUM> torque_sensor_raw; // Torque sensor data

#ifdef SLAVE_FT
  std::array<std::array<uint8_t, 6>, FT_SENSOR_NUM>
      ft_torque_raw;                                              // Force/Torque sensor torque data
  std::array<std::array<uint8_t, 6>, FT_SENSOR_NUM> ft_force_raw; // Force/Torque sensor force data
#endif
} AnybusCommunicator;
// ETHERCAT_DATA_H_

#endif
