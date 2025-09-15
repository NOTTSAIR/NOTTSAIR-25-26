cmake_minimum_required(VERSION 3.10)
project(imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-O2 -Wall -Wextra)

add_executable(imu src/imu.cpp)              # <-- NOT imu_viewer.cpp
ament_target_dependencies(imu rclcpp sensor_msgs)
target_link_libraries(imu ${OpenCV_LIBS})
install(TARGETS imu DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

