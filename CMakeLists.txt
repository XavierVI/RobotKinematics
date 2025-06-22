cmake_minimum_required(VERSION 3.10)

project(RobotKinematics VERSION 1.0)

enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "/usr/local/include/" ${CMAKE_PREFIX_PATH})

set(MUJOCO_PATH "${PROJECT_SOURCE_DIR}/../mujoco-3.3.3")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_PATH}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_PATH}/lib")

find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_PATH}/lib)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_executable(main src/main.cpp)

add_executable(sim src/sim.cpp)
target_include_directories(sim PRIVATE ${MUJOCO_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})
target_link_directories(sim PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(sim
  mujoco
  ${GLFW_LIBRARIES}  
)

set_target_properties(sim PROPERTIES
    INSTALL_RPATH "${MUJOCO_LIB_DIR}"
)

install(TARGETS main sim DESTINATION ${PROJECT_SOURCE_DIR}/bin)

