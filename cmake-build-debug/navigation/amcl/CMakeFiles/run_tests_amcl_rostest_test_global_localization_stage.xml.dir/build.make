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

# Utility rule file for run_tests_amcl_rostest_test_global_localization_stage.xml.

# Include any custom commands dependencies for this target.
include navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/compiler_depend.make

# Include the progress variables for this target.
include navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/progress.make

navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/amcl && ../../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/run_tests.py /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/test_results/amcl/rostest-test_global_localization_stage.xml "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/cx/brainnavi/ai_robot_ranger/src/navigation/amcl --package=amcl --results-filename test_global_localization_stage.xml --results-base-dir \"/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/test_results\" /home/cx/brainnavi/ai_robot_ranger/src/navigation/amcl/test/global_localization_stage.xml "

run_tests_amcl_rostest_test_global_localization_stage.xml: navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml
run_tests_amcl_rostest_test_global_localization_stage.xml: navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/build.make
.PHONY : run_tests_amcl_rostest_test_global_localization_stage.xml

# Rule to build all files generated by this target.
navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/build: run_tests_amcl_rostest_test_global_localization_stage.xml
.PHONY : navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/build

navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/clean:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/amcl && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/cmake_clean.cmake
.PHONY : navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/clean

navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/depend:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/ai_robot_ranger/src /home/cx/brainnavi/ai_robot_ranger/src/navigation/amcl /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/amcl /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/amcl/CMakeFiles/run_tests_amcl_rostest_test_global_localization_stage.xml.dir/depend

