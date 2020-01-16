# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reef_msgs: 11 messages, 0 services")

set(MSG_I_FLAGS "-Ireef_msgs:/home/humberto/catkin_ws/src/reef_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reef_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" ""
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" "reef_msgs/XYEstimate:reef_msgs/ZEstimate:std_msgs/Header"
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" ""
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" ""
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" "reef_msgs/DesiredVector:std_msgs/Header"
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" "geometry_msgs/Vector3:geometry_msgs/TwistWithCovarianceStamped:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/TwistWithCovariance"
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" ""
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" ""
)

get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_custom_target(_reef_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reef_msgs" "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" "reef_msgs/ZDebugEstimate:reef_msgs/XYDebugEstimate:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)
_generate_msg_cpp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(reef_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reef_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reef_msgs_generate_messages reef_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_cpp _reef_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reef_msgs_gencpp)
add_dependencies(reef_msgs_gencpp reef_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reef_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)
_generate_msg_eus(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(reef_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(reef_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(reef_msgs_generate_messages reef_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_eus _reef_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reef_msgs_geneus)
add_dependencies(reef_msgs_geneus reef_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reef_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)
_generate_msg_lisp(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(reef_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(reef_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(reef_msgs_generate_messages reef_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_lisp _reef_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reef_msgs_genlisp)
add_dependencies(reef_msgs_genlisp reef_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reef_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)
_generate_msg_nodejs(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(reef_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(reef_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(reef_msgs_generate_messages reef_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_nodejs _reef_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reef_msgs_gennodejs)
add_dependencies(reef_msgs_gennodejs reef_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reef_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)
_generate_msg_py(reef_msgs
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg"
  "${MSG_I_FLAGS}"
  "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg;/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(reef_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reef_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reef_msgs_generate_messages reef_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncVerifyEstimates.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/ZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredState.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DeltaToVel.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/SyncEstimateError.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/DesiredVector.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/humberto/catkin_ws/src/reef_msgs/msg/XYZDebugEstimate.msg" NAME_WE)
add_dependencies(reef_msgs_generate_messages_py _reef_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reef_msgs_genpy)
add_dependencies(reef_msgs_genpy reef_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reef_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reef_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(reef_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reef_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reef_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(reef_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(reef_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reef_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(reef_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(reef_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reef_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(reef_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(reef_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reef_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(reef_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reef_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
