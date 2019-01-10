# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "airhockey_main: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iairhockey_main:/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(airhockey_main_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_custom_target(_airhockey_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "airhockey_main" "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(airhockey_main
  "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/airhockey_main
)

### Generating Services

### Generating Module File
_generate_module_cpp(airhockey_main
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/airhockey_main
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(airhockey_main_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(airhockey_main_generate_messages airhockey_main_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_dependencies(airhockey_main_generate_messages_cpp _airhockey_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(airhockey_main_gencpp)
add_dependencies(airhockey_main_gencpp airhockey_main_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS airhockey_main_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(airhockey_main
  "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/airhockey_main
)

### Generating Services

### Generating Module File
_generate_module_eus(airhockey_main
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/airhockey_main
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(airhockey_main_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(airhockey_main_generate_messages airhockey_main_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_dependencies(airhockey_main_generate_messages_eus _airhockey_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(airhockey_main_geneus)
add_dependencies(airhockey_main_geneus airhockey_main_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS airhockey_main_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(airhockey_main
  "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/airhockey_main
)

### Generating Services

### Generating Module File
_generate_module_lisp(airhockey_main
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/airhockey_main
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(airhockey_main_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(airhockey_main_generate_messages airhockey_main_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_dependencies(airhockey_main_generate_messages_lisp _airhockey_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(airhockey_main_genlisp)
add_dependencies(airhockey_main_genlisp airhockey_main_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS airhockey_main_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(airhockey_main
  "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/airhockey_main
)

### Generating Services

### Generating Module File
_generate_module_nodejs(airhockey_main
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/airhockey_main
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(airhockey_main_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(airhockey_main_generate_messages airhockey_main_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_dependencies(airhockey_main_generate_messages_nodejs _airhockey_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(airhockey_main_gennodejs)
add_dependencies(airhockey_main_gennodejs airhockey_main_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS airhockey_main_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(airhockey_main
  "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/airhockey_main
)

### Generating Services

### Generating Module File
_generate_module_py(airhockey_main
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/airhockey_main
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(airhockey_main_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(airhockey_main_generate_messages airhockey_main_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alvin/robotic_airhockey_ws/src/airhockey_main/msg/ArmAngles.msg" NAME_WE)
add_dependencies(airhockey_main_generate_messages_py _airhockey_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(airhockey_main_genpy)
add_dependencies(airhockey_main_genpy airhockey_main_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS airhockey_main_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/airhockey_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/airhockey_main
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(airhockey_main_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/airhockey_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/airhockey_main
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(airhockey_main_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/airhockey_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/airhockey_main
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(airhockey_main_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/airhockey_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/airhockey_main
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(airhockey_main_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/airhockey_main)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/airhockey_main\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/airhockey_main
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(airhockey_main_generate_messages_py std_msgs_generate_messages_py)
endif()
