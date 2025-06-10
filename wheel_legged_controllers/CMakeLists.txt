cmake_minimum_required(VERSION 3.10)
project(wheel_legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        wheel_legged_common
        wheel_legged_interface
        wheel_legged_wbc
        wheel_legged_estimation
        wheel_legged_dummy
        controller_interface
        ocs2_self_collision_visualization
        angles
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        wheel_legged_common
        wheel_legged_interface
        wheel_legged_wbc
        wheel_legged_estimation
        wheel_legged_dummy
        controller_interface
        ocs2_self_collision_visualization
        angles
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/WheelLeggedController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(wheel_legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(wheel_legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(wheel_legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(wheel_legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

add_executable(cheat_controller_node
        src/CheatControllerNode.cpp
        )
add_dependencies(cheat_controller_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cheat_controller_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(cheat_controller_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(normal_controller_node
        src/NormalControllerNode.cpp
        )
add_dependencies(normal_controller_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(normal_controller_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(normal_controller_node PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} wheel_legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES Wheel_legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
