cmake_minimum_required(VERSION 3.8)
project(robin_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/node_encoder_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_encoder
)


ament_package()
