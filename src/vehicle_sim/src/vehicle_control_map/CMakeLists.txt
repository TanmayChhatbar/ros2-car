cmake_minimum_required(VERSION 3.12)
project(vehicle_control_map LANGUAGES CXX)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

SET(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${CMAKE_SOURCE_DIR}/build")

find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_LOADED)
    message(STATUS "Compiler Path: ${CMAKE_CXX_COMPILER}")
    message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# set the Ipopt library and include directory
set(LIBRARY_DIRS /usr/local/lib)
set(VEHICLE_SIM_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../include)

# find the following packages
find_package(Threads REQUIRED)

# include header directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CODE_GEN_INCLUDE_DIR})

# library directories
link_directories(${LIBRARY_DIRS})

# find all the header files
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
file(GLOB HEADER_FILES_H ${CMAKE_SOURCE_DIR}/include/*.h)

# casadi + ipopt test
add_executable(genControlMap_test 
    genControlMap_test.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2D.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2DConfig.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2DData.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/VehicleInput.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/TireConfig.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/OptSolver.cpp
)
include_directories(${VEHICLE_SIM_INCLUDE_DIR})
