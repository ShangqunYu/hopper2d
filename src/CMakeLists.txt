cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(hopper2d VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
configure_file(Hopper2dConfig.h.in Hopper2dConfig.h)
# add the executable
add_executable(hopper2d main.cpp)

add_subdirectory(render)

target_include_directories(hopper2d PUBLIC
                           "${PROJECT_BINARY_DIR}"
			   PUBLIC render)
target_link_libraries(hopper2d animate)
