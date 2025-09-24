# # Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.10)
project(mylib VERSION 0.0.0)

# # Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

# find SOEM - more flexible in Nix environment
if(DEFINED ENV{SOEM_PATH})
  set(SOEM_DIR $ENV{SOEM_PATH})
  message(STATUS "Using SOEM_PATH from environment: ${SOEM_DIR}")
elseif(DEFINED SOEM_DIR)
  message(STATUS "Using SOEM_DIR from CMake: ${SOEM_DIR}")
else()
  message(FATAL_ERROR "SOEM not found. Please set SOEM_PATH or SOEM_DIR")
endif()

# debug SOEM installation structure
message(STATUS "=== SOEM Debug Info ===")
message(STATUS "SOEM_DIR: ${SOEM_DIR}")
execute_process(
  COMMAND find ${SOEM_DIR} -name "*.h" -o -name "*.a" -o -name "*.so"
  OUTPUT_VARIABLE SOEM_FILES
  ERROR_QUIET
)
message(STATUS "SOEM files found:\n${SOEM_FILES}")

# check possible SOEM include paths
set(POSSIBLE_INCLUDE_DIRS
  "${SOEM_DIR}/include"
  "${SOEM_DIR}/include/soem"
  "${SOEM_DIR}/soem"
)

set(SOEM_INCLUDE_DIR "")

foreach(DIR ${POSSIBLE_INCLUDE_DIRS})
  if(EXISTS "${DIR}/ethercat.h")
    set(SOEM_INCLUDE_DIR ${DIR})
    message(STATUS "Found SOEM headers at: ${DIR}")
    break()
  endif()
endforeach()

if(NOT SOEM_INCLUDE_DIR)
  message(FATAL_ERROR "SOEM headers (ethercat.h) not found in any expected location")
endif()

# check possible SOEM library paths
set(POSSIBLE_LIB_PATHS
  "${SOEM_DIR}/lib/libsoem.a"
  "${SOEM_DIR}/lib/libsoem.so"
  "${SOEM_DIR}/libsoem.a"
  "${SOEM_DIR}/libsoem.so"
)

set(SOEM_LIBRARY "")

foreach(LIB ${POSSIBLE_LIB_PATHS})
  if(EXISTS ${LIB})
    set(SOEM_LIBRARY ${LIB})
    message(STATUS "Found SOEM library at: ${LIB}")
    break()
  endif()
endforeach()

if(NOT SOEM_LIBRARY)
  message(FATAL_ERROR "SOEM library not found in any expected location")
endif()

message(STATUS "Final SOEM settings:")
message(STATUS "  Include dir: ${SOEM_INCLUDE_DIR}")
message(STATUS "  Library: ${SOEM_LIBRARY}")
message(STATUS "========================")

find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
message(STATUS "Using pinocchio at ${pinocchio_INCLUDE_DIRS}")

# # Find source and header files to create executable binary file
file(GLOB_RECURSE MYLIB_SOURCES

  # actuator/src/*.cpp
  ethercat/src/*.cpp

  # math/src/*.cpp
  utilities/src/*.cpp

  # controller/src/*.cpp
)

# # Define library, connecting source files & Link libraries
add_library(mylib SHARED ${MYLIB_SOURCES})
add_library(mylib::mylib ALIAS mylib)

target_link_libraries(mylib PUBLIC
  ${SOEM_LIBRARY}
  yaml-cpp
  pinocchio::pinocchio
  Eigen3::Eigen
)

target_include_directories(mylib
  PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ethercat/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utilities/include>
  $<BUILD_INTERFACE:${SOEM_INCLUDE_DIR}>

  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:${SOEM_INCLUDE_DIR}>

  # $<BUILD_INTERFACE:${pinocchio_INCLUDE_DIRS}>
  # $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

include(CMakePackageConfigHelpers)

# only write package version / configure package file if template exists
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mylibConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mylibConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mylibConfig.cmake"
  INSTALL_DESTINATION lib/cmake/mylib
)

install(TARGETS mylib
  EXPORT mylibTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION include
)

# install(EXPORT mylibTargets
# FILE mylibConfig.cmake
# NAMESPACE mylib::
# DESTINATION lib/cmake/mylib)
# install(EXPORT mylibTargets FILE mylibTargets.cmake DESTINATION lib/cmake/mylib)
install(EXPORT mylibTargets
  FILE mylibTargets.cmake
  NAMESPACE mylib::
  DESTINATION lib/cmake/mylib
)

# Install the configured package files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mylibConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mylibConfigVersion.cmake"
  DESTINATION lib/cmake/mylib
)

# install(DIRECTORY actuator/include/
# DESTINATION include)
install(DIRECTORY ethercat/include/
  DESTINATION include)

install(DIRECTORY utilities/include/
  DESTINATION include)

# install(DIRECTORY math/include/
# DESTINATION include)

# ! actual files should be exist in the directory
# install(DIRECTORY controller/include/
# DESTINATION include)
