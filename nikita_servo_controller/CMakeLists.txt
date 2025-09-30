cmake_minimum_required(VERSION 3.8)
project(nikita_servo_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nikita_interfaces REQUIRED)

add_executable(node_servo_controller
  src/node_servo_controller.cpp
  src/servo_controller.cpp
  src/bus_servo_protocol.cpp
)
ament_target_dependencies(node_servo_controller rclcpp std_msgs nikita_interfaces)

include_directories(
  include/nikita_servo_controller/
)

install(TARGETS
  node_servo_controller
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
