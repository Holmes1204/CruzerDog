cmake_minimum_required(VERSION 3.0.2)
project(main_ctrl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
#set(CMAKE_CXX_FLAGS_RELEASE " -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)


find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        rospy
        unitree_legged_msgs
        wtr_serial_msg
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

include_directories(
        # /usr/include/eigen3/
        # /usr/local/include/osqp/
        # /usr/local/include/osqp-eigen/
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable( 
        main_ctrl
        src/main_ctrl.cpp
        # src/test.cpp
        src/FSM/FiniteStateMachine.cpp
        src/FSM/FSM_data.cpp
        src/FSM/FSM_tpcl.cpp
        src/FSM/State_Worker/State_Worker.cpp
        src/FSM/State_Worker/Stand_Worker.cpp
        src/FSM/State_Worker/Steady_Worker.cpp
        src/FSM/State_Worker/Locomotion_Worker.cpp
        src/Leg_Control/Foot_Swing_Trajectory.cpp
        src/Leg_Control/Gait.cpp
        src/Leg_Control/Leg_Control.cpp
        src/Robot/Robot.cpp
        src/Convex_MPC/MConvexMPC.cpp
        src/State_Estimator/state_estimator.cpp
        src/State_Estimator/A1BasicEKF.cpp
        )

target_link_libraries(
        main_ctrl
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        osqp::osqp
)