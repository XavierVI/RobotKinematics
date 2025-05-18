cmake_minimum_required(VERSION 3.10)

project(RobotKinematics VERSION 1.0)

find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
    add_executable(modern_robotics src/modern_robotics.cpp)
    target_link_libraries(modern_robotics Eigen3::Eigen)
else()
    message(FATAL_ERROR "Eigen3 library not found.")
endif()




