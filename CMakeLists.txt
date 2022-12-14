cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(hopper2d VERSION 1.0)

# eigen
include_directories("/usr/local/include/eigen3/")
# casadi
find_package(casadi REQUIRED) 
include_directories("/usr/local/include/casadi/")

find_package(SFML COMPONENTS system window graphics network audio REQUIRED)
include_directories("usr/include/SFML/")

find_package(lcm REQUIRED)
include_directories("/usr/local/include/lcm/") 

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


# add the executable
add_executable(hopper2d main.cpp ) 
 

target_include_directories(hopper2d PUBLIC
                           "${PROJECT_BINARY_DIR}"
			                PUBLIC render
                            PUBLIC simulation
                            PUBLIC utils
                            PUBLIC matlab_gen
                            PUBLIC cart-pole)

                          

message(STATUS "${PROJECT_BINARY_DIR}")


target_link_libraries(hopper2d hopper_simulate utils casadi matlab_gen cart-pole_system)
#target_link_libraries(hopper2d hopper_simulate dmToEigen casadi sfml-graphics)


