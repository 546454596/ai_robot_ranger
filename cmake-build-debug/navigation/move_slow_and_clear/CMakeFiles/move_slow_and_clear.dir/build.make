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
include navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/depend.make
# Include the progress variables for this target.
include navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/flags.make

navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o: navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/flags.make
navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o: ../navigation/move_slow_and_clear/src/move_slow_and_clear.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o -c /home/cx/brainnavi/ai_robot_ranger/src/navigation/move_slow_and_clear/src/move_slow_and_clear.cpp

navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.i"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/ai_robot_ranger/src/navigation/move_slow_and_clear/src/move_slow_and_clear.cpp > CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.i

navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.s"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/ai_robot_ranger/src/navigation/move_slow_and_clear/src/move_slow_and_clear.cpp -o CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.s

# Object files for target move_slow_and_clear
move_slow_and_clear_OBJECTS = \
"CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o"

# External object files for target move_slow_and_clear
move_slow_and_clear_EXTERNAL_OBJECTS =

/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/src/move_slow_and_clear.cpp.o
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/build.make
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_thread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_chrono.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_system.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_date_time.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_atomic.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /home/cx/brainnavi/ai_robot_ranger/devel/lib/liblayers.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_system.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_thread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_chrono.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_date_time.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_atomic.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/liblaser_geometry.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf2_ros.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libactionlib.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf2.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libclass_loader.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/libPocoFoundation.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroslib.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librospack.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroscpp.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librostime.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libcpp_common.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /home/cx/brainnavi/ai_robot_ranger/devel/lib/libcostmap_2d.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_system.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_thread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_chrono.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_date_time.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/local/lib/libboost_atomic.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/liblaser_geometry.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /home/cx/brainnavi/ai_robot_ranger/devel/lib/libvoxel_grid.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libclass_loader.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/libPocoFoundation.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroslib.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librospack.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/liborocos-kdl.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf2_ros.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libactionlib.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libtf2.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroscpp.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/librostime.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /opt/ros/melodic/lib/libcpp_common.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so: navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/move_slow_and_clear.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/build: /home/cx/brainnavi/ai_robot_ranger/devel/lib/libmove_slow_and_clear.so
.PHONY : navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/build

navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/clean:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear && $(CMAKE_COMMAND) -P CMakeFiles/move_slow_and_clear.dir/cmake_clean.cmake
.PHONY : navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/clean

navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/depend:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/ai_robot_ranger/src /home/cx/brainnavi/ai_robot_ranger/src/navigation/move_slow_and_clear /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/move_slow_and_clear/CMakeFiles/move_slow_and_clear.dir/depend

