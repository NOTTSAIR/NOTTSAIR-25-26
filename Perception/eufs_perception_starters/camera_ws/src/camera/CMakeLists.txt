cmake_minimum_required(VERSION 3.10)
project(camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-O2 -Wall -Wextra)

add_executable(camera src/camera.cpp)        # <-- NOT cam_four_view.cpp
ament_target_dependencies(camera rclcpp sensor_msgs image_transport cv_bridge)
target_link_libraries(camera ${OpenCV_LIBS})
install(TARGETS camera DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()

