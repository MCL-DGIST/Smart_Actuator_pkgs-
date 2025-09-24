#include "ecat_device_control_objects.hpp"

const std::unordered_map<uint8_t, std::string> mode_of_operation_str = {
  { 1, "Profile Position" },
  { 2, "Velocity" },
  { 3, "Profile Velocity" },
  { 4, "Profile Torque" },
  { 6, "Homing" },
  { 7, "Interpolated Position" },
  { 8, "Cyclic Sync Position" },
  { 9, "Cyclic Sync Velocity" },
  { 10, "Cyclic Sync Torque" },
};

std::string getModeOfOperationStr(uint8_t mode_of_operation)
{
  auto it = mode_of_operation_str.find(mode_of_operation);
  return it != mode_of_operation_str.end() ? it->second : "Unknown";
}

const std::unordered_map<uint16_t, std::string> controlword_str = {
  { 6, "Shutdown" },            //
  { 7, "Switch On" },           //
  { 0, "Disable Voltage" },     //
  { 2, "Quick Stop" },          //
  { 14, "Disable Operation" },  //
  { 15, "Enable Operation" },   //
  { 128, "Fault Reset" },       //
};

std::string getControlwordStr(uint16_t controlword)
{
  auto it = controlword_str.find(controlword);
  return it != controlword_str.end() ? it->second : "Unknown";
}