# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_SOURCE_DIR = /home/david/webots/synthetic_data/controllers/camera_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/webots/synthetic_data/controllers/camera_controller

# Include any dependencies generated for this target.
include CMakeFiles/test_program.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_program.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_program.dir/flags.make

CMakeFiles/test_program.dir/test_program.cpp.o: CMakeFiles/test_program.dir/flags.make
CMakeFiles/test_program.dir/test_program.cpp.o: test_program.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_program.dir/test_program.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_program.dir/test_program.cpp.o -c /home/david/webots/synthetic_data/controllers/camera_controller/test_program.cpp

CMakeFiles/test_program.dir/test_program.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_program.dir/test_program.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/webots/synthetic_data/controllers/camera_controller/test_program.cpp > CMakeFiles/test_program.dir/test_program.cpp.i

CMakeFiles/test_program.dir/test_program.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_program.dir/test_program.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/webots/synthetic_data/controllers/camera_controller/test_program.cpp -o CMakeFiles/test_program.dir/test_program.cpp.s

CMakeFiles/test_program.dir/src/supervisor_node.cpp.o: CMakeFiles/test_program.dir/flags.make
CMakeFiles/test_program.dir/src/supervisor_node.cpp.o: src/supervisor_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_program.dir/src/supervisor_node.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_program.dir/src/supervisor_node.cpp.o -c /home/david/webots/synthetic_data/controllers/camera_controller/src/supervisor_node.cpp

CMakeFiles/test_program.dir/src/supervisor_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_program.dir/src/supervisor_node.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/webots/synthetic_data/controllers/camera_controller/src/supervisor_node.cpp > CMakeFiles/test_program.dir/src/supervisor_node.cpp.i

CMakeFiles/test_program.dir/src/supervisor_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_program.dir/src/supervisor_node.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/webots/synthetic_data/controllers/camera_controller/src/supervisor_node.cpp -o CMakeFiles/test_program.dir/src/supervisor_node.cpp.s

CMakeFiles/test_program.dir/src/camera_utils.cpp.o: CMakeFiles/test_program.dir/flags.make
CMakeFiles/test_program.dir/src/camera_utils.cpp.o: src/camera_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test_program.dir/src/camera_utils.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_program.dir/src/camera_utils.cpp.o -c /home/david/webots/synthetic_data/controllers/camera_controller/src/camera_utils.cpp

CMakeFiles/test_program.dir/src/camera_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_program.dir/src/camera_utils.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/webots/synthetic_data/controllers/camera_controller/src/camera_utils.cpp > CMakeFiles/test_program.dir/src/camera_utils.cpp.i

CMakeFiles/test_program.dir/src/camera_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_program.dir/src/camera_utils.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/webots/synthetic_data/controllers/camera_controller/src/camera_utils.cpp -o CMakeFiles/test_program.dir/src/camera_utils.cpp.s

CMakeFiles/test_program.dir/src/transform_utils.cpp.o: CMakeFiles/test_program.dir/flags.make
CMakeFiles/test_program.dir/src/transform_utils.cpp.o: src/transform_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test_program.dir/src/transform_utils.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_program.dir/src/transform_utils.cpp.o -c /home/david/webots/synthetic_data/controllers/camera_controller/src/transform_utils.cpp

CMakeFiles/test_program.dir/src/transform_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_program.dir/src/transform_utils.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/webots/synthetic_data/controllers/camera_controller/src/transform_utils.cpp > CMakeFiles/test_program.dir/src/transform_utils.cpp.i

CMakeFiles/test_program.dir/src/transform_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_program.dir/src/transform_utils.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/webots/synthetic_data/controllers/camera_controller/src/transform_utils.cpp -o CMakeFiles/test_program.dir/src/transform_utils.cpp.s

# Object files for target test_program
test_program_OBJECTS = \
"CMakeFiles/test_program.dir/test_program.cpp.o" \
"CMakeFiles/test_program.dir/src/supervisor_node.cpp.o" \
"CMakeFiles/test_program.dir/src/camera_utils.cpp.o" \
"CMakeFiles/test_program.dir/src/transform_utils.cpp.o"

# External object files for target test_program
test_program_EXTERNAL_OBJECTS =

test_program: CMakeFiles/test_program.dir/test_program.cpp.o
test_program: CMakeFiles/test_program.dir/src/supervisor_node.cpp.o
test_program: CMakeFiles/test_program.dir/src/camera_utils.cpp.o
test_program: CMakeFiles/test_program.dir/src/transform_utils.cpp.o
test_program: CMakeFiles/test_program.dir/build.make
test_program: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2
test_program: /usr/lib/x86_64-linux-gnu/libboost_system.so
test_program: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test_program: CMakeFiles/test_program.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable test_program"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_program.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_program.dir/build: test_program

.PHONY : CMakeFiles/test_program.dir/build

CMakeFiles/test_program.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_program.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_program.dir/clean

CMakeFiles/test_program.dir/depend:
	cd /home/david/webots/synthetic_data/controllers/camera_controller && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/webots/synthetic_data/controllers/camera_controller /home/david/webots/synthetic_data/controllers/camera_controller /home/david/webots/synthetic_data/controllers/camera_controller /home/david/webots/synthetic_data/controllers/camera_controller /home/david/webots/synthetic_data/controllers/camera_controller/CMakeFiles/test_program.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_program.dir/depend

