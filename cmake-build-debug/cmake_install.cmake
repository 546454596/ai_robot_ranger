# Install script for directory: /home/cx/brainnavi/ai_robot_ranger/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/setup.bash"
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/setup.sh"
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/setup.zsh"
    "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/gtest/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot-master/ai_robot_localization/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot-master/ai_robot_sensors/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navigation/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot_ranger_mini/ranger_mini_control/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot_ranger_mini/ranger_mini_gazebo/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/slam_toolbox/slam_toolbox_msgs/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/map_server/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot-master/ai_robot_msgs/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros_msgs/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot-master/ai_robot_navigation/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/darknet_ros/darknet_ros/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/amcl/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/fake_localization/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot_ranger_mini/ranger_mini/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/slam_toolbox/slam_toolbox/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/voxel_grid/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/costmap_2d/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot_clean/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/nav_core/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/base_local_planner/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/carrot_planner/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/clear_costmap_recovery/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/dwa_local_planner/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_slow_and_clear/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/navfn/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/global_planner/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/rotate_recovery/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/navigation/move_base/cmake_install.cmake")
  include("/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/ai_robot_nav_demo/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/cx/brainnavi/ai_robot_ranger/src/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
