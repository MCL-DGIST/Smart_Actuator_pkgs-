/**
 * @file ecat_device_control_objects.hpp
 * @author Jeongwoo Hong (jwhong1209@gmail.com)
 * @brief Data Structure for EtherCAT Controlword and Statusword
 * @version 0.0.0
 * @date 2025-06-11
 * @copyright Copyright (c) 2025
 * TODO: Check if values are correct. Add more if necessary
 * TODO: Priority of status would be necessary to be specified e.g. Fault > Warning > ...
 */

#ifndef ETHERCAT_CONTROL_STATUS_MAP_HPP_
#define ETHERCAT_CONTROL_STATUS_MAP_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

/* Mode of Operation Enum Class, Hash map & Getter function */
// TODO: Check if values are correct. Add more if necessary
enum class ModeOfOperationEnum
{
  PROFILE_POSITION_MODE            = 1,
  VELOCITY_MODE                    = 2,
  PROFILE_VELOCITY_MODE            = 3,
  PROFILE_TORQUE_MODE              = 4,
  HOMING_MODE                      = 6,
  INTERPOLATED_POSITION_MODE       = 7,
  CYCLIC_SYNCHRONOUS_POSITION_MODE = 8,
  CYCLIC_SYNCHRONOUS_VELOCITY_MODE = 9,
  CYCLIC_SYNCHRONOUS_TORQUE_MODE   = 10,
};

extern const std::unordered_map<uint8_t, std::string> mode_of_operation_str;
std::string getModeOfOperationStr(uint8_t mode_of_operation);

/* Controlword Enum Class, Hash map & Getter function */
enum class ControlWordEnum
{
  // QUICK_STOP        = 2,
  SHUTDOWN          = 6,
  SWITCH_ON         = 7,
  DISABLE_VOLTAGE   = 0,
  DISABLE_OPERATION = 14,
  ENABLE_OPERATION  = 15,
  FAULT_RESET       = 128,
};

extern const std::unordered_map<uint16_t, std::string> controlword_str;
std::string getControlwordStr(uint16_t controlword);

/* Statusword Bit Masks */
constexpr uint16_t STATUS_BIT_READY_TO_SWITCH_ON = 0x0001;  // bit 0
constexpr uint16_t STATUS_BIT_SWITCHED_ON        = 0x0002;  // bit 1
constexpr uint16_t STATUS_BIT_OPERATION_ENABLED  = 0x0004;  // bit 2
constexpr uint16_t STATUS_BIT_FAULT              = 0x0008;  // bit 3
constexpr uint16_t STATUS_BIT_VOLTAGE_ENABLED    = 0x0010;  // bit 4
constexpr uint16_t STATUS_BIT_QUICK_STOP         = 0x0020;  // bit 5
constexpr uint16_t STATUS_BIT_SWITCH_ON_DISABLED = 0x0040;  // bit 6
constexpr uint16_t STATUS_BIT_WARNING            = 0x0080;  // bit 7
constexpr uint16_t STATUS_BIT_REMOTE             = 0x0200;  // bit 9
constexpr uint16_t STATUS_BIT_TARGET_REACHED     = 0x0400;  // bit 10
constexpr uint16_t STATUS_BIT_INTERNAL_LIMIT     = 0x0800;  // bit 11

/* Helper functions for Statusword bit checking */
inline bool isNotReadyToSwitchOn(uint16_t statusword)
{                                                  // X 0 X X 0 0 0 0
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == 0;
}

inline bool isSwitchOnDisabled(uint16_t statusword)
{                                                  // X 1 X X 0 0 0 0
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == STATUS_BIT_SWITCH_ON_DISABLED;
}

inline bool isReadyToSwitchOn(uint16_t statusword)
{                                                  // X 0 1 X 0 0 0 1
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_QUICK_STOP |          // bit 5
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == (STATUS_BIT_QUICK_STOP | STATUS_BIT_READY_TO_SWITCH_ON);
}

inline bool isSwitchedOn(uint16_t statusword)
{                                                  // X 0 1 X 0 0 1 1
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_QUICK_STOP |          // bit 5
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) ==
         (STATUS_BIT_QUICK_STOP | STATUS_BIT_SWITCHED_ON | STATUS_BIT_READY_TO_SWITCH_ON);
}

inline bool isOperationEnabled(uint16_t statusword)
{                                                  // X 0 1 X 0 1 1 1
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_QUICK_STOP |          // bit 5
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == (STATUS_BIT_QUICK_STOP | STATUS_BIT_OPERATION_ENABLED |
                                 STATUS_BIT_SWITCHED_ON | STATUS_BIT_READY_TO_SWITCH_ON);
}

inline bool isQuickStopActive(uint16_t statusword)
{                                                  // X 0 0 X 0 1 1 1
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_QUICK_STOP |          // bit 5
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) ==
         (STATUS_BIT_OPERATION_ENABLED | STATUS_BIT_SWITCHED_ON | STATUS_BIT_READY_TO_SWITCH_ON);
}

inline bool isFaultReactionActive(uint16_t statusword)
{                                                  // X 0 X X 1 1 1 1
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == (STATUS_BIT_FAULT | STATUS_BIT_OPERATION_ENABLED |
                                 STATUS_BIT_SWITCHED_ON | STATUS_BIT_READY_TO_SWITCH_ON);
}

inline bool isFault(uint16_t statusword)
{                                                  // X 0 X X 1 0 0 0
  uint16_t mask = STATUS_BIT_SWITCH_ON_DISABLED |  // bit 6
                  STATUS_BIT_FAULT |               // bit 3
                  STATUS_BIT_OPERATION_ENABLED |   // bit 2
                  STATUS_BIT_SWITCHED_ON |         // bit 1
                  STATUS_BIT_READY_TO_SWITCH_ON;   // bit 0
  return (statusword & mask) == STATUS_BIT_FAULT;
}

inline bool isMainPowerOn(uint16_t statusword)
{                                              // X X X 1 X X X X
  uint16_t mask = STATUS_BIT_VOLTAGE_ENABLED;  // bit 4
  return (statusword & mask) == STATUS_BIT_VOLTAGE_ENABLED;
}

inline bool isWarningOccurred(uint16_t statusword)
{                                      // 1 X X X X X X X
  uint16_t mask = STATUS_BIT_WARNING;  // bit 7
  return (statusword & mask) == STATUS_BIT_WARNING;
}

/* Getter functions for Statusword */
inline std::string getStatuswordStr(uint16_t statusword)
{
  if (isNotReadyToSwitchOn(statusword))
    return "NotReadySwitchOn";
  if (isSwitchOnDisabled(statusword))
    return "SwitchOnDisabled";
  if (isReadyToSwitchOn(statusword))
    return "ReadySwitchOn";
  if (isSwitchedOn(statusword))
    return "SwitchedOn";
  if (isOperationEnabled(statusword))
    return "OperationEnabled";
  if (isQuickStopActive(statusword))
    return "QuickStopActive";
  if (isFaultReactionActive(statusword))
    return "FaultReactionActive";
  if (isFault(statusword))
    return "Fault";
  // if (isMainPowerOn(statusword)) // ! These may conflict with other statuswords
  //   return "Main Power On";
  // if (isWarningOccurred(statusword)) // ! These may conflict with other statuswords
  //   return "Warning";
  return "Unknown";
}

#endif  // ETHERCAT_CONTROL_STATUS_MAP_HPP_