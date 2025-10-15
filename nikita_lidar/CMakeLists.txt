cmake_minimum_required(VERSION 3.8)
project(nikita_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nikita_interfaces REQUIRED)

## set sources
set(SOURCES
    src/LIDARLite_RaspberryPi_Library/src/lidarlite_v3.cpp
)

add_executable(node_lidar
  src/node_lidar.cpp
  ${SOURCES}
)
ament_target_dependencies(node_lidar rclcpp std_msgs geometry_msgs std_srvs nikita_interfaces)

include_directories(
  include/nikita_lidar/
  src/LIDARLite_RaspberryPi_Library/include/
)

install(TARGETS
  node_lidar
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
