cmake_minimum_required(VERSION 3.10.2)
project(ecn_auv_lab)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(yaml-cpp)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DAUV_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/params/")

ament_auto_add_executable(gz2ekf src/gz2ekf.cpp)
target_link_libraries(gz2ekf yaml-cpp)
ament_auto_add_executable(waypoints src/waypoints.cpp)
target_link_libraries(waypoints yaml-cpp)

install(DIRECTORY urdf launch params DESTINATION share/${PROJECT_NAME})

ament_auto_package()
