include_directories(${CMAKE_SOURCE_DIR}/render)
message(STATUS "${CMAKE_SOURCE_DIR}")
add_library(hopper_simulate hopper_simulate.cpp dynamics.cpp )
target_link_libraries(hopper_simulate animate)