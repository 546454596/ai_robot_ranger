# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /home/cx/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/212.5457.51/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cx/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/212.5457.51/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cx/brainnavi/ai_robot_ranger/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug

# Include any dependencies generated for this target.
include navigation/costmap_2d/CMakeFiles/static_tests.dir/depend.make
# Include the progress variables for this target.
include navigation/costmap_2d/CMakeFiles/static_tests.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/costmap_2d/CMakeFiles/static_tests.dir/flags.make

navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o: navigation/costmap_2d/CMakeFiles/static_tests.dir/flags.make
navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o: ../navigation/costmap_2d/test/static_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/static_tests.dir/test/static_tests.cpp.o -c /home/cx/brainnavi/ai_robot_ranger/src/navigation/costmap_2d/test/static_tests.cpp

navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/static_tests.dir/test/static_tests.cpp.i"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/ai_robot_ranger/src/navigation/costmap_2d/test/static_tests.cpp > CMakeFiles/static_tests.dir/test/static_tests.cpp.i

navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/static_tests.dir/test/static_tests.cpp.s"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/ai_robot_ranger/src/navigation/costmap_2d/test/static_tests.cpp -o CMakeFiles/static_tests.dir/test/static_tests.cpp.s

# Object files for target static_tests
static_tests_OBJECTS = \
"CMakeFiles/static_tests.dir/test/static_tests.cpp.o"

# External object files for target static_tests
static_tests_EXTERNAL_OBJECTS =

devel/lib/costmap_2d/static_tests: navigation/costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o
devel/lib/costmap_2d/static_tests: navigation/costmap_2d/CMakeFiles/static_tests.dir/build.make
devel/lib/costmap_2d/static_tests: devel/lib/liblayers.so
devel/lib/costmap_2d/static_tests: gtest/googlemock/gtest/libgtest.so
devel/lib/costmap_2d/static_tests: devel/lib/libcostmap_2d.so
devel/lib/costmap_2d/static_tests: /usr/local/lib/libboost_system.so
devel/lib/costmap_2d/static_tests: /usr/local/lib/libboost_thread.so
devel/lib/costmap_2d/static_tests: /usr/local/lib/libboost_chrono.so
devel/lib/costmap_2d/static_tests: /usr/local/lib/libboost_date_time.so
devel/lib/costmap_2d/static_tests: /usr/local/lib/libboost_atomic.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liblaser_geometry.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/costmap_2d/static_tests: /usr/lib/libPocoFoundation.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroslib.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librospack.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2_ros.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libactionlib.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2.so
devel/lib/costmap_2d/static_tests: devel/lib/libvoxel_grid.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librostime.so
devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/costmap_2d/static_tests: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/costmap_2d/static_tests: navigation/costmap_2d/CMakeFiles/static_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../devel/lib/costmap_2d/static_tests"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/static_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/costmap_2d/CMakeFiles/static_tests.dir/build: devel/lib/costmap_2d/static_tests
.PHONY : navigation/costmap_2d/CMakeFiles/static_tests.dir/build

navigation/costmap_2d/CMakeFiles/static_tests.dir/clean:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d && $(CMAKE_COMMAND) -P CMakeFiles/static_tests.dir/cmake_clean.cmake
.PHONY : navigation/costmap_2d/CMakeFiles/static_tests.dir/clean

navigation/costmap_2d/CMakeFiles/static_tests.dir/depend:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/ai_robot_ranger/src /home/cx/brainnavi/ai_robot_ranger/src/navigation/costmap_2d /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d/CMakeFiles/static_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/costmap_2d/CMakeFiles/static_tests.dir/depend

