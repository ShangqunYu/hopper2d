include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/utils)
pybind11_add_module(hopper2dWrapper hopper2dWrapper.cpp)
target_link_libraries(hopper2dWrapper PRIVATE hopper_simulate animate lcm casadi)
