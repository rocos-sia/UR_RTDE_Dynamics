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
include CMakeFiles/forcemode_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/forcemode_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/forcemode_example.dir/flags.make

CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o: CMakeFiles/forcemode_example.dir/flags.make
CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o: ../forcemode_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k/UR_RTDE_Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o -c /home/k/UR_RTDE_Examples/forcemode_example.cpp

CMakeFiles/forcemode_example.dir/forcemode_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/forcemode_example.dir/forcemode_example.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k/UR_RTDE_Examples/forcemode_example.cpp > CMakeFiles/forcemode_example.dir/forcemode_example.cpp.i

CMakeFiles/forcemode_example.dir/forcemode_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/forcemode_example.dir/forcemode_example.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k/UR_RTDE_Examples/forcemode_example.cpp -o CMakeFiles/forcemode_example.dir/forcemode_example.cpp.s

# Object files for target forcemode_example
forcemode_example_OBJECTS = \
"CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o"

# External object files for target forcemode_example
forcemode_example_EXTERNAL_OBJECTS =

forcemode_example: CMakeFiles/forcemode_example.dir/forcemode_example.cpp.o
forcemode_example: CMakeFiles/forcemode_example.dir/build.make
forcemode_example: /usr/local/lib/librtde.so.1.5.6
forcemode_example: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
forcemode_example: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
forcemode_example: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
forcemode_example: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
forcemode_example: CMakeFiles/forcemode_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/k/UR_RTDE_Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable forcemode_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/forcemode_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/forcemode_example.dir/build: forcemode_example

.PHONY : CMakeFiles/forcemode_example.dir/build

CMakeFiles/forcemode_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/forcemode_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/forcemode_example.dir/clean

CMakeFiles/forcemode_example.dir/depend:
	cd /home/k/UR_RTDE_Examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/k/UR_RTDE_Examples /home/k/UR_RTDE_Examples /home/k/UR_RTDE_Examples/build /home/k/UR_RTDE_Examples/build /home/k/UR_RTDE_Examples/build/CMakeFiles/forcemode_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/forcemode_example.dir/depend

