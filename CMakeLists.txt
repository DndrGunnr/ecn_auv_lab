cmake_minimum_required(VERSION 3.10.2)
project(ecn_auv_lab)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)

ament_auto_add_executable(gz2ekf src/gz2ekf.cpp)
ament_auto_add_executable(control src/control.cpp)

find_package(ament_cmake QUIET)

install(DIRECTORY urdf launch params DESTINATION share/${PROJECT_NAME})

ament_auto_package()
