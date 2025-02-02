cmake_minimum_required(VERSION 3.5)
project(image_streaming_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs OpenCV cv_bridge)

install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
