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

# Utility rule file for clean_test_results_navfn.

# Include any custom commands dependencies for this target.
include navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/compiler_depend.make

# Include the progress variables for this target.
include navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/progress.make

navigation/navfn/test/CMakeFiles/clean_test_results_navfn:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navfn/test && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/test_results/navfn

clean_test_results_navfn: navigation/navfn/test/CMakeFiles/clean_test_results_navfn
clean_test_results_navfn: navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/build.make
.PHONY : clean_test_results_navfn

# Rule to build all files generated by this target.
navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/build: clean_test_results_navfn
.PHONY : navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/build

navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/clean:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navfn/test && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_navfn.dir/cmake_clean.cmake
.PHONY : navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/clean

navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/depend:
	cd /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/ai_robot_ranger/src /home/cx/brainnavi/ai_robot_ranger/src/navigation/navfn/test /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navfn/test /home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/navfn/test/CMakeFiles/clean_test_results_navfn.dir/depend

