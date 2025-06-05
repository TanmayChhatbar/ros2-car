cmake_minimum_required(VERSION 3.12)
project(vehicle_control_map LANGUAGES CXX)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

SET(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${CMAKE_SOURCE_DIR}/build")

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wpedantic -Wall -Wextra -pthread -Werror")
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

set(VEHICLE_SIM_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../include)
set(SOLVER_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

# include header directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include)

add_executable(genControlMap
    genControlMap.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2D.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2DConfig.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/Vehicle2DData.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/VehicleInput.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/TireConfig.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/CSVWriter.cpp
    ${VEHICLE_SIM_INCLUDE_DIR}/CSVReader.cpp
    ${SOLVER_INCLUDE_DIR}/BruteSolver.cpp
)
include_directories(${VEHICLE_SIM_INCLUDE_DIR})
if(OpenMP_CXX_FOUND)
    target_link_libraries(genControlMap PRIVATE OpenMP::OpenMP_CXX)
    target_compile_options(genControlMap PRIVATE ${OpenMP_CXX_FLAGS})
endif()
