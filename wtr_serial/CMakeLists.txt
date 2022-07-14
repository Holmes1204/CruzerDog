cmake_minimum_required(VERSION 3.0.2)
project(wtr_serial)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

# 链接头文件
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(serial)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        wtr_serial_msg
        )

catkin_package(
        CATKIN_DEPENDS wtr_serial_msg roscpp
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
        ${serial_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(wtr_serial  src/wtr_serial.cpp)

add_dependencies(${PROJECT_NAME} wtr_serial_msg_gencpp)

target_link_libraries(wtr_serial ${THIRD_PARTY_LIBS})
