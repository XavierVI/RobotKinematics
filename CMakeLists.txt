cmake_minimum_required(VERSION 3.10)

project(RobotKinematics VERSION 1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "/usr/local/include/" ${CMAKE_PREFIX_PATH})

add_executable(main src/main.cpp)

install(TARGETS main DESTINATION ${PROJECT_SOURCE_DIR}/bin)

