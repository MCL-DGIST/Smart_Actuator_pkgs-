cmake_minimum_required(VERSION 3.16)
project(Xenomai4 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ------------------------------------------------------------
# SOEM 경로 직접 지정
# ------------------------------------------------------------
set(SOEM_INCLUDE_DIR /usr/SOEM/soem)
set(SOEM_LIB_DIR /usr/SOEM/build)

find_library(SOEM_LIBRARY
  NAMES soem
  PATHS ${SOEM_LIB_DIR}
)
find_path(EVLINCLUDE_DIR
  NAMES evl/thread.h
  PATHS /usr/local/include /usr/include
)

find_path(EVLUAPI_DIR
  NAMES evl/syscall-abi.h
  PATHS /usr/local/include/uapi /usr/include/uapi
)

find_library(EVLLIBRARY
  NAMES evl
  PATHS /usr/local/lib /usr/lib
)


if(NOT SOEM_LIBRARY)
  message(FATAL_ERROR "SOEM library not found in ${SOEM_LIB_DIR}")
endif()


find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(joy REQUIRED)
find_package(mylib REQUIRED CONFIG)

pkg_check_modules(EVL REQUIRED evl)
find_library(BPF_LIBRARY NAMES bpf)
if(BPF_LIBRARY)
  get_filename_component(BPF_LIBDIR ${BPF_LIBRARY} DIRECTORY)
endif()
# ------------------------------------------------------------
# Include/Link
# ------------------------------------------------------------
include_directories(
  ${EVLINCLUDE_DIR}
  ${EVLUAPI_DIR}
  ${SOEM_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/ethercat
  ${CMAKE_SOURCE_DIR}/evl_func
  ${CMAKE_CURRENT_SOURCE_DIR}/state_manager
  ${CMAKE_CURRENT_SOURCE_DIR}/assets/model/urdf

  ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory

  /usr/include/eigen3
  /usr/SOEM/osal
  /usr/SOEM/osal/linux
  /usr/SOEM/oshw/linux

  
)

link_directories(
  ${EVL_LIBRARY_DIRS}

)

# ------------------------------------------------------------
# Sources
# ------------------------------------------------------------
set(PROJECT_SOURCES
  main.cc
  ethercat/ecat_func.cc
  Controller.cc
  evl_func/rt_func.cc
  Actuator.cc
  Trajectory/Trajectory.cc
  Integrate.cc
  Kinematics.cc
  filter.cc
  ${CMAKE_SOURCE_DIR}/../../mylib/ethercat/src/ecat_hardware_interface.cpp
  ${CMAKE_SOURCE_DIR}/../../mylib/ethercat/src/ecat_device_control_objects.cpp
)


# ------------------------------------------------------------
# Target
# ------------------------------------------------------------
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${EVL_LIBRARIES}
  ${SOEM_LIBRARY}
  pthread
  rt
  ${BPF_LIBRARY}
)



if(BPF_LIBRARY)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
    LINK_OPTIONS "-Wl,-rpath,${BPF_LIBDIR}")
endif()


# # ROS 2 인터페이스
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mcl_quadruped_msgs REQUIRED)
add_executable(ros_interface_node
  ${CMAKE_CURRENT_SOURCE_DIR}/ros_interface/ros_interface_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ros_interface/ros_interface.cpp
)
ament_target_dependencies(ros_interface_node
  rclcpp
  std_msgs
  sensor_msgs
  mcl_quadruped_msgs
)
# target_compile_options(ros_interface_node PRIVATE ${EVL_CFLAGS})
target_include_directories(ros_interface_node
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/state_manager
  ${CMAKE_CURRENT_SOURCE_DIR}/ros_interface
)
target_link_libraries(ros_interface_node
  rt
  pthread
  mylib::mylib
)
# ----------------------------------------
# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ros_interface_node
  # ros_interface_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BPF_LIBRARY)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
    LINK_OPTIONS "-Wl,-rpath,${BPF_LIBDIR}")
endif()


ament_package()

