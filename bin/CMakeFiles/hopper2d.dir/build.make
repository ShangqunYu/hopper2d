# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.23.0-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.23.0-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/simon/Repository/hopper2d/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/simon/Repository/hopper2d/bin

# Include any dependencies generated for this target.
include CMakeFiles/hopper2d.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/hopper2d.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/hopper2d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hopper2d.dir/flags.make

CMakeFiles/hopper2d.dir/main.cpp.o: CMakeFiles/hopper2d.dir/flags.make
CMakeFiles/hopper2d.dir/main.cpp.o: /home/simon/Repository/hopper2d/src/main.cpp
CMakeFiles/hopper2d.dir/main.cpp.o: CMakeFiles/hopper2d.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/simon/Repository/hopper2d/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hopper2d.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/hopper2d.dir/main.cpp.o -MF CMakeFiles/hopper2d.dir/main.cpp.o.d -o CMakeFiles/hopper2d.dir/main.cpp.o -c /home/simon/Repository/hopper2d/src/main.cpp

CMakeFiles/hopper2d.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hopper2d.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/simon/Repository/hopper2d/src/main.cpp > CMakeFiles/hopper2d.dir/main.cpp.i

CMakeFiles/hopper2d.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hopper2d.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/simon/Repository/hopper2d/src/main.cpp -o CMakeFiles/hopper2d.dir/main.cpp.s

# Object files for target hopper2d
hopper2d_OBJECTS = \
"CMakeFiles/hopper2d.dir/main.cpp.o"

# External object files for target hopper2d
hopper2d_EXTERNAL_OBJECTS =

hopper2d: CMakeFiles/hopper2d.dir/main.cpp.o
hopper2d: CMakeFiles/hopper2d.dir/build.make
hopper2d: render/libanimate.a
hopper2d: CMakeFiles/hopper2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/simon/Repository/hopper2d/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable hopper2d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hopper2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hopper2d.dir/build: hopper2d
.PHONY : CMakeFiles/hopper2d.dir/build

CMakeFiles/hopper2d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hopper2d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hopper2d.dir/clean

CMakeFiles/hopper2d.dir/depend:
	cd /home/simon/Repository/hopper2d/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/simon/Repository/hopper2d/src /home/simon/Repository/hopper2d/src /home/simon/Repository/hopper2d/bin /home/simon/Repository/hopper2d/bin /home/simon/Repository/hopper2d/bin/CMakeFiles/hopper2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hopper2d.dir/depend

