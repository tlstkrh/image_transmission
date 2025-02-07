cmake_minimum_required(VERSION 3.5)
project(image_transmission)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


add_executable(stream_publish src/stream_publish.cpp)
ament_target_dependencies(stream_publish rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(subscribe src/subscribe.cpp)
ament_target_dependencies(subscribe rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(gs_pub testcode/gs_pub.cpp)
ament_target_dependencies(gs_pub rclcpp sensor_msgs OpenCV cv_bridge)


install(TARGETS
  stream_publish
  subscribe
  gs_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
