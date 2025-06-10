cmake_minimum_required(VERSION 3.20)
project(wheel_legged_hardwares)
#设置编译的标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  wheel_legged_controllers
)

catkin_package(
)

#使用EtherCAT通讯的必要文件
add_subdirectory(EtherCAT)
#EtherCAT控制必要引入的文件
include_directories(
        /usr/include/python3.8
        EtherCAT/cpp-readline/src
        EtherCAT/app
        EtherCAT/osal
        EtherCAT/osal/linux
        EtherCAT/soem
        EtherCAT/oshw/linux
)


#链接到eigen3矩阵运算库
include_directories(/usr/include/eigen3)

#自己写的库函数的必要文件
include_directories(
    include/${PROJECT_NAME}
    EtherCAT/app
    xspublic
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  xspublic
)


# Hardware node
link_directories(src/lib)

add_executable(HardwareToRos
  src/main.cpp
)

target_sources(HardwareToRos PUBLIC
              ${ETHERCAT_SOURCE}
		          EtherCAT/app/math_ops.c
		          EtherCAT/app/transmit.cpp
		          EtherCAT/app/motor_control.cpp
              src/mti_receive_data.cpp
              src/hardwares.cpp
)

target_link_libraries(HardwareToRos
  ${catkin_LIBRARIES}
  soem motor
  xscontroller -lxstypes xscommon -lpthread -lrt -ldl
)

# Bridge node

add_executable(RosToOcs2_hardwares
  src/RosToOcs2_hardwares.cpp
)
add_dependencies(RosToOcs2_hardwares
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(RosToOcs2_hardwares
  ${catkin_LIBRARIES}
)



