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
include darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/depend.make
# Include the progress variables for this target.
include darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/progress.make

# Include the compile flags for this target's objects.
include darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/flags.make

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/flags.make
darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o: ../darknet_ros/darknet_ros/test/test_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o -c /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/test_main.cpp

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.i"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/test_main.cpp > CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.i

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.s"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/test_main.cpp -o CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.s

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/flags.make
darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o: ../darknet_ros/darknet_ros/test/ObjectDetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o -c /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/ObjectDetection.cpp

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.i"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/ObjectDetection.cpp > CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.i

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.s"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros/test/ObjectDetection.cpp -o CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.s

# Object files for target darknet_ros_object_detection-test
darknet_ros_object_detection__test_OBJECTS = \
"CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o" \
"CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o"

# External object files for target darknet_ros_object_detection-test
darknet_ros_object_detection__test_EXTERNAL_OBJECTS =

devel/lib/darknet_ros/darknet_ros_object_detection-test: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/test_main.cpp.o
devel/lib/darknet_ros/darknet_ros_object_detection-test: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/test/ObjectDetection.cpp.o
devel/lib/darknet_ros/darknet_ros_object_detection-test: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/build.make
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libSM.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libICE.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libX11.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libXext.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: gtest/googlemock/gtest/libgtest.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libactionlib.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libnodeletlib.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libbondcpp.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libuuid.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/libPocoFoundation.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libroslib.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/librospack.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libroscpp.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/librosconsole.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/librostime.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/darknet_ros/darknet_ros_object_detection-test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/darknet_ros/darknet_ros_object_detection-test: darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../devel/lib/darknet_ros/darknet_ros_object_detection-test"
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/darknet_ros_object_detection-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/build: devel/lib/darknet_ros/darknet_ros_object_detection-test
.PHONY : darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/build

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/clean:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros && $(CMAKE_COMMAND) -P CMakeFiles/darknet_ros_object_detection-test.dir/cmake_clean.cmake
.PHONY : darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/clean

darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/depend:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/ai_robot_ranger/src /home/cx/brainnavi/ai_robot_ranger/src/darknet_ros/darknet_ros /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : darknet_ros/darknet_ros/CMakeFiles/darknet_ros_object_detection-test.dir/depend

