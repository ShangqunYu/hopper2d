cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(hopper2d VERSION 1.0)

add_subdirectory(machinelearning/pybind11)
set(CMAKE_CXX_FLAGS "-fPIC")

# find_package(Python 3.10.11 COMPONENTS Interpreter Development REQUIRED)

include_directories("/usr/local/include/")
# eigen
include_directories("/usr/local/include/eigen3/")
# casadi
find_package(casadi REQUIRED) 
include_directories("/usr/local/include/casadi/")
# include_directories("/usr/local/include/")

#find_package(SFML COMPONENTS system window graphics network audio REQUIRED)
#include_directories("usr/include/SFML/")

find_package(lcm REQUIRED)
include_directories("/usr/local/include/lcm/") 

# set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/lcm-types/cpp)
include_directories(${CMAKE_SOURCE_DIR}/cart-pole)
include_directories(${CMAKE_BINARY_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
configure_file(Hopper2dConfig.h.in Hopper2dConfig.h)

add_subdirectory(simulation)
add_subdirectory(utils)
add_subdirectory(render)
add_subdirectory(matlab_gen)
add_subdirectory(cart-pole)
add_subdirectory(machinelearning)
add_subdirectory(optimization)

# add the executable
add_executable(hopper2d main.cpp ) 
 

target_include_directories(hopper2d PUBLIC
                           "${PROJECT_BINARY_DIR}"
			                PUBLIC render
                            PUBLIC simulation
                            PUBLIC utils
                            PUBLIC matlab_gen
                            PUBLIC cart-pole
                            PUBLIC optimization
                            PUBLIC machinelearning)

                          

message(STATUS "${PROJECT_BINARY_DIR}")


target_link_libraries(hopper2d hopper_simulate utils casadi matlab_gen cart-pole_system animate optimize)
#target_link_libraries(hopper2d hopper_simulate dmToEigen casadi sfml-graphics)


