cmake_minimum_required(VERSION 3.10)
project(gps)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-O2 -Wall -Wextra)

add_executable(gps src/gps.cpp)              # <-- NOT lidar or anything else
ament_target_dependencies(gps rclcpp sensor_msgs nav_msgs)
target_link_libraries(gps ${OpenCV_LIBS})
install(TARGETS gps DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

