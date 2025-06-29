cmake_minimum_required(VERSION 3.0.2)
project(wheel_legged_dummy)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  wheel_legged_interface
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
  Boost
  pinocchio
)

###########
## Build ##
###########

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/WheelLeggedRobotVisualizer.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# # DDP-MPC node for legged robot
# add_executable(wheel_legged_robot_ddp_mpc
#   src/WheelLeggedRobotDdpMpcNode.cpp
# )
# add_dependencies(wheel_legged_robot_ddp_mpc
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(wheel_legged_robot_ddp_mpc
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# target_compile_options(wheel_legged_robot_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(wheel_legged_robot_sqp_mpc
  src/WheelLeggedRobotSqpMpcNode.cpp
)
add_dependencies(wheel_legged_robot_sqp_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wheel_legged_robot_sqp_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(wheel_legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# ## IPM-MPC node for legged robot
# add_executable(wheel_legged_robot_ipm_mpc
#   src/WheelLeggedRobotIpmMpcNode.cpp
# )
# add_dependencies(wheel_legged_robot_ipm_mpc
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(wheel_legged_robot_ipm_mpc
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# target_compile_options(wheel_legged_robot_ipm_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(wheel_legged_robot_dummy
  src/WheelLeggedRobotDummyNode.cpp
)
add_dependencies(wheel_legged_robot_dummy
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wheel_legged_robot_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(wheel_legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(wheel_legged_robot_target
  src/WheelLeggedRobotPoseCommandNode.cpp
)
add_dependencies(wheel_legged_robot_target
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wheel_legged_robot_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(wheel_legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(wheel_legged_robot_gait_command
  src/WheelLeggedRobotGaitCommandNode.cpp
)
add_dependencies(wheel_legged_robot_gait_command
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wheel_legged_robot_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(wheel_legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target Wheel_legged_dummy")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
      #  wheel_legged_robot_ddp_mpc
       wheel_legged_robot_sqp_mpc
      #  wheel_legged_robot_ipm_mpc
       wheel_legged_robot_dummy
       wheel_legged_robot_target
       wheel_legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS
    # wheel_legged_robot_ddp_mpc
    wheel_legged_robot_sqp_mpc
    # wheel_legged_robot_ipm_mpc
    wheel_legged_robot_dummy
    wheel_legged_robot_target
    wheel_legged_robot_gait_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
