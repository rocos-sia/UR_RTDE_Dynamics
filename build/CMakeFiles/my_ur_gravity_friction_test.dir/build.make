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
CMAKE_SOURCE_DIR = /home/k/UR_RTDE_Examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/k/UR_RTDE_Examples/build

# Include any dependencies generated for this target.
include CMakeFiles/my_ur_gravity_friction_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_ur_gravity_friction_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_ur_gravity_friction_test.dir/flags.make

CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o: CMakeFiles/my_ur_gravity_friction_test.dir/flags.make
CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o: ../my_ur_gravity_friction_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k/UR_RTDE_Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o -c /home/k/UR_RTDE_Examples/my_ur_gravity_friction_test.cpp

CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k/UR_RTDE_Examples/my_ur_gravity_friction_test.cpp > CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.i

CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k/UR_RTDE_Examples/my_ur_gravity_friction_test.cpp -o CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.s

# Object files for target my_ur_gravity_friction_test
my_ur_gravity_friction_test_OBJECTS = \
"CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o"

# External object files for target my_ur_gravity_friction_test
my_ur_gravity_friction_test_EXTERNAL_OBJECTS =

my_ur_gravity_friction_test: CMakeFiles/my_ur_gravity_friction_test.dir/my_ur_gravity_friction_test.cpp.o
my_ur_gravity_friction_test: CMakeFiles/my_ur_gravity_friction_test.dir/build.make
my_ur_gravity_friction_test: /usr/local/lib/librtde.so.1.5.6
my_ur_gravity_friction_test: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
my_ur_gravity_friction_test: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
my_ur_gravity_friction_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
my_ur_gravity_friction_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
my_ur_gravity_friction_test: CMakeFiles/my_ur_gravity_friction_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/k/UR_RTDE_Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable my_ur_gravity_friction_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_ur_gravity_friction_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_ur_gravity_friction_test.dir/build: my_ur_gravity_friction_test

.PHONY : CMakeFiles/my_ur_gravity_friction_test.dir/build

CMakeFiles/my_ur_gravity_friction_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_ur_gravity_friction_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_ur_gravity_friction_test.dir/clean

CMakeFiles/my_ur_gravity_friction_test.dir/depend:
	cd /home/k/UR_RTDE_Examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/k/UR_RTDE_Examples /home/k/UR_RTDE_Examples /home/k/UR_RTDE_Examples/build /home/k/UR_RTDE_Examples/build /home/k/UR_RTDE_Examples/build/CMakeFiles/my_ur_gravity_friction_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_ur_gravity_friction_test.dir/depend

