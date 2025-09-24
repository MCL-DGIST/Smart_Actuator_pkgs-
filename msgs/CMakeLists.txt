cmake_minimum_required(VERSION 3.10)
project(mcl_quadruped_msgs)

# set C/C++ Standard
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# set compile options
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(python_cmake_module REQUIRED)

## Generate messages in the 'msg' folder
set(msg_files
  "msg/GuiCommand.msg"
  "msg/ControllerState.msg"
  "msg/ActuatorState.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  # ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)


# Macro for ament package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
