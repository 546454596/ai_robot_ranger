cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(ai_robot_navigation)

SET(CMAKE_CXX_FLAGS -std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

find_package(Eigen3 REQUIRED)
find_package(Boost)
find_package(GeographicLib REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_srvs
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  image_transport
  ai_robot_msgs
)


FIND_PACKAGE( PCL REQUIRED COMPONENTS common io filters visualization)
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#FIND_PACKAGE( octomap REQUIRED )
#INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )
find_package( mavros_msgs REQUIRED )
find_package(OpenCV REQUIRED)


catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES drone_sensor
   CATKIN_DEPENDS  message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
 include/controller
#/usr/local/include/VisualSLAM
${PCL_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS}
)
#set(MoSLAM_LIB
#        libVisualSLAM.a
#        GL
#        GLU
#        lapack
#        blas
#        m
#)

## Declare a cpp executable

add_executable(brain_node
               src/brain/main.cpp
               src/brain/AIBrain.cpp
               src/brain/posControl.cpp
               src/brain/postrajectory.cpp
               src/brain/ARDrone.cpp
               src/brain/drone_object_ros.cpp
               src/brain/Pioneer3AT.cpp
               src/findpath/findpathsrm.cpp
               src/pointtopoint/PointToPoint.cpp
               #src/obstacle_avoid/p3atObsAvoid.cpp
               #src/brain/ObstacleAvoid.cpp
               #src/brain/AstarPathPlanning2D.cpp
               include/controller/PIDController.cpp
               include/controller/pid_fuzzy.cpp
               include/controller/ADRC.cpp
               include/controller/ESO.cpp
               include/assistMath.cpp
)

add_executable(pos_ekf_node
               src/pos_ekf/main.cpp
               src/pos_ekf/pose_ekf.cpp
               src/pos_ekf/vicon.cpp
               include/assistMath.cpp
)

add_executable(obsavoid_node
               src/obstacle_avoid/main.cpp
               src/obstacle_avoid/p3atObsAvoid.cpp
               src/brain/Pioneer3AT.cpp
               include/assistMath.cpp
)

add_executable(pointcloud_viewer_node
    src/obstacle_avoid/pointcloud_viewer.cpp
)

add_executable(gennodemap_node
    src/gennodemap/src/ros_main.cpp
    src/gennodemap/src/keyframehandle.cpp
)

ADD_EXECUTABLE(testfindpath
    src/findpath/testfindpath.cpp
    src/findpath/findpathsrm.cpp
    include/assistMath.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries( brain_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
target_link_libraries( pos_ekf_node
    ${EIGEN_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries( obsavoid_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
target_link_libraries( pointcloud_viewer_node
    ${EIGEN_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(gennodemap_node
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(testfindpath
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}   
)

#target_link_libraries( altitudeoverride_node
#    ${catkin_LIBRARIES}
#)

set_target_properties(  brain_node
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)

set_target_properties(  pos_ekf_node
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(  obsavoid_node
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(  pointcloud_viewer_node
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
add_executable(setDesination_client src/findpath/setDestinationClient.cpp)
target_link_libraries(setDesination_client ${catkin_LIBRARIES})
#add_dependencies(setDesination_client beginner_tutorials_gencpp)
