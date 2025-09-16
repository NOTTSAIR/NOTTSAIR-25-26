cmake_minimum_required(VERSION 3.8)
project(static_a)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)

add_executable(static_inspection_a src/static_inspection_a.cpp)
ament_target_dependencies(static_inspection_a rclcpp ackermann_msgs eufs_msgs)

# Make the binary available to ros2 run
install(TARGETS static_inspection_a
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config so ros2 launch can find them
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

