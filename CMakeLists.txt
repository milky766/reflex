cmake_minimum_required(VERSION 3.1)
project(control_board)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

##Uncomment for Debug Build
set(DCMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

find_package (Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(BCM2835 REQUIRED)

SET(SOURCES
        src/lib/ad5360.cpp
        src/lib/ad7616.cpp
        src/lib/ad7730.cpp
        src/lib/circular_buffer.hpp
        src/lib/comparator.cpp
        src/lib/control_board.cpp
        src/lib/DataFitting.cpp
        src/lib/iir_filter.cpp
        src/lib/interneuron.cpp
        src/lib/lower_controller.cpp
        src/lib/muscle.cpp
        src/lib/pid_controller.cpp
        src/lib/raspberry_pi_gpio.cpp
        src/lib/raspberry_pi_spi.cpp
        src/lib/rate_loop.cpp
        src/lib/rt_ave.cpp
        src/lib/top_controller.cpp
        )

include_directories(
        include
        ${BCM2835_INCLUDE_DIRS}
)

add_library(control_board ${SOURCES})
target_link_libraries(control_board
        ${BCM2835_LIBRARIES}
)

#add additional executables

add_executable(hysteresis_test src/hysteresis_test.cpp)
target_link_libraries(hysteresis_test
        rt
        control_board
        ${Boost_LIBRARIES}
        )

#add_executable(0715_motor_control src/0715_motor_control.cpp)
#target_link_libraries(0715_motor_control
#        rt
#        control_board
#        ${Boost_LIBRARIES}
#        )

add_executable(reflex_control src/reflex_control.cpp)
target_link_libraries(reflex_control
        rt
        control_board
        ${Boost_LIBRARIES}
        )

add_executable(Tensionmapping src/Tensionmapping.cpp)
target_link_libraries(Tensionmapping
        rt
        control_board
        ${Boost_LIBRARIES}
        )
        

