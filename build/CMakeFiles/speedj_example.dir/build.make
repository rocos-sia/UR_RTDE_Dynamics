# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build

# Include any dependencies generated for this target.
include CMakeFiles/speedj_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/speedj_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/speedj_example.dir/flags.make

CMakeFiles/speedj_example.dir/speedj_example.cpp.o: CMakeFiles/speedj_example.dir/flags.make
CMakeFiles/speedj_example.dir/speedj_example.cpp.o: ../speedj_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/speedj_example.dir/speedj_example.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/speedj_example.dir/speedj_example.cpp.o -c /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/speedj_example.cpp

CMakeFiles/speedj_example.dir/speedj_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/speedj_example.dir/speedj_example.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/speedj_example.cpp > CMakeFiles/speedj_example.dir/speedj_example.cpp.i

CMakeFiles/speedj_example.dir/speedj_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/speedj_example.dir/speedj_example.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/speedj_example.cpp -o CMakeFiles/speedj_example.dir/speedj_example.cpp.s

# Object files for target speedj_example
speedj_example_OBJECTS = \
"CMakeFiles/speedj_example.dir/speedj_example.cpp.o"

# External object files for target speedj_example
speedj_example_EXTERNAL_OBJECTS =

speedj_example: CMakeFiles/speedj_example.dir/speedj_example.cpp.o
speedj_example: CMakeFiles/speedj_example.dir/build.make
speedj_example: /usr/local/lib/librtde.so.1.5.6
speedj_example: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
speedj_example: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
speedj_example: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
speedj_example: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
speedj_example: CMakeFiles/speedj_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable speedj_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/speedj_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/speedj_example.dir/build: speedj_example

.PHONY : CMakeFiles/speedj_example.dir/build

CMakeFiles/speedj_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/speedj_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/speedj_example.dir/clean

CMakeFiles/speedj_example.dir/depend:
	cd /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build /home/k/UR_RTDE_Dynamics/UR_RTDE_Dynamics/build/CMakeFiles/speedj_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/speedj_example.dir/depend

