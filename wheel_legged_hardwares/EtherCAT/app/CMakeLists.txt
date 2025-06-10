cmake_minimum_required(VERSION 3.20)
project(master_stack_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -pthread" )


set(SOURCES motor_control.cpp
            motor_control.h
            motor_control_mf9025.cpp
            motor_control_mf9025.h
            motor_control_dm8006.cpp
            motor_control_dm8006.h
            math_ops.c
            math_ops.h
            transmit.cpp
            transmit.h
            config.h
            command.cpp
            command.h
            plot.cpp
            plot.h
)

# Find Python，主要是用matplotlibcpp库使用
find_package(Python3 COMPONENTS Development NumPy)
# Include directories，主要是用matplotlibcpp库使用
include_directories(${Python3_INCLUDE_DIRS})


LINK_LIBRARIES(m)
add_library(motor ${SOURCES})
target_link_libraries(motor soem -lpython3.8)

add_executable(master_stack_test main.cpp)
target_link_libraries(master_stack_test motor soem)

#链接Python库，主要是用matplotlibcpp库使用
target_link_libraries(master_stack_test
        Python3::Python Python3::NumPy
)

