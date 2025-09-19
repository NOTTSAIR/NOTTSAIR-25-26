cmake_minimum_required(VERSION 3.10)
project(yolo_infer)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-O2 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# LibTorch
# Set Torch_DIR via environment or CMake arg:
#   -DTorch_DIR=/path/to/libtorch/share/cmake/Torch
find_package(Torch REQUIRED)

add_executable(yolo_node src/yolo_infer.cpp)
ament_target_dependencies(yolo_node
  rclcpp sensor_msgs image_transport cv_bridge vision_msgs
)
target_link_libraries(yolo_node ${OpenCV_LIBS} "${TORCH_LIBRARIES}")

# Suppress rpath warnings with LibTorch
set_property(TARGET yolo_node PROPERTY CXX_STANDARD 17)
if (MSVC)
  set_property(TARGET yolo_node PROPERTY VS_GLOBAL_EnableMicrosoftCodeAnalysis false)
endif()

install(TARGETS yolo_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

