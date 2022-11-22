# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ai_robot_msgs: 2 messages, 2 services")

set(MSG_I_FLAGS "-Iai_robot_msgs:/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ai_robot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_custom_target(_ai_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ai_robot_msgs" "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" ""
)

get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_custom_target(_ai_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ai_robot_msgs" "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/PointStamped"
)

get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_custom_target(_ai_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ai_robot_msgs" "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" "nav_msgs/Path:geometry_msgs/PointStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_custom_target(_ai_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ai_robot_msgs" "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
)
_generate_msg_cpp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
)

### Generating Services
_generate_srv_cpp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
)
_generate_srv_cpp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
)

### Generating Module File
_generate_module_cpp(ai_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ai_robot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ai_robot_msgs_generate_messages ai_robot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_cpp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_cpp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_cpp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_cpp _ai_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ai_robot_msgs_gencpp)
add_dependencies(ai_robot_msgs_gencpp ai_robot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ai_robot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
)
_generate_msg_eus(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
)

### Generating Services
_generate_srv_eus(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
)
_generate_srv_eus(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
)

### Generating Module File
_generate_module_eus(ai_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ai_robot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ai_robot_msgs_generate_messages ai_robot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_eus _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_eus _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_eus _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_eus _ai_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ai_robot_msgs_geneus)
add_dependencies(ai_robot_msgs_geneus ai_robot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ai_robot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
)
_generate_msg_lisp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
)

### Generating Services
_generate_srv_lisp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
)
_generate_srv_lisp(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
)

### Generating Module File
_generate_module_lisp(ai_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ai_robot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ai_robot_msgs_generate_messages ai_robot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_lisp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_lisp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_lisp _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_lisp _ai_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ai_robot_msgs_genlisp)
add_dependencies(ai_robot_msgs_genlisp ai_robot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ai_robot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
)
_generate_msg_nodejs(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
)

### Generating Services
_generate_srv_nodejs(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
)
_generate_srv_nodejs(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
)

### Generating Module File
_generate_module_nodejs(ai_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ai_robot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ai_robot_msgs_generate_messages ai_robot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_nodejs _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_nodejs _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_nodejs _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_nodejs _ai_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ai_robot_msgs_gennodejs)
add_dependencies(ai_robot_msgs_gennodejs ai_robot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ai_robot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
)
_generate_msg_py(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
)

### Generating Services
_generate_srv_py(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
)
_generate_srv_py(ai_robot_msgs
  "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
)

### Generating Module File
_generate_module_py(ai_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ai_robot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ai_robot_msgs_generate_messages ai_robot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/restart_nav.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_py _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/TopoMetric.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_py _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/msg/MpTraj.msg" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_py _ai_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cx/brainnavi/ai_robot_ranger/src/ai_robot-master/ai_robot_msgs/srv/set_destination.srv" NAME_WE)
add_dependencies(ai_robot_msgs_generate_messages_py _ai_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ai_robot_msgs_genpy)
add_dependencies(ai_robot_msgs_genpy ai_robot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ai_robot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ai_robot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ai_robot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ai_robot_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(ai_robot_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ai_robot_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ai_robot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ai_robot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ai_robot_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(ai_robot_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ai_robot_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ai_robot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ai_robot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ai_robot_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(ai_robot_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ai_robot_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ai_robot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ai_robot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ai_robot_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(ai_robot_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ai_robot_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ai_robot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ai_robot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ai_robot_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(ai_robot_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ai_robot_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
