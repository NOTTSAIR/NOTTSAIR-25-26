cmake_minimum_required(VERSION 3.10)
project(yolo_depth)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-O2 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

# LibTorch
# Set Torch_DIR before configure if needed, for example:
# export Torch_DIR=/opt/libtorch/share/cmake/Torch
find_package(Torch REQUIRED)

# Help the built binary find LibTorch at runtime
get_filename_component(TORCH_PREFIX "${Torch_DIR}/../.." ABSOLUTE)
list(APPEND CMAKE_BUILD_RPATH "${TORCH_PREFIX}/lib")
list(APPEND CMAKE_INSTALL_RPATH "${TORCH_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(yolo_depth_node src/yolo_depth_node.cpp)
ament_target_dependencies(yolo_depth_node
  rclcpp sensor_msgs vision_msgs cv_bridge OpenCV message_filters
)
target_link_libraries(yolo_depth_node "${TORCH_LIBRARIES}" ${OpenCV_LIBS})

install(TARGETS yolo_depth_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
