cmake_minimum_required(VERSION 3.8)
project(nikita_movement)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nikita_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED) # nlohmann_json -> sudo apt install nlohmann-json3-dev


## set sources
set(SOURCES
  src/requester/requester.cpp
  src/requester/kinematics.cpp
  src/requester/gaitcontroller.cpp
  src/requester/lowpassfilter.cpp
  src/requester/actionpackagesparser.cpp
  src/action/action_executor.cpp
  src/handler/servohandler.cpp
)

add_executable(node_movement
  src/node_movement.cpp
  ${SOURCES}
)
ament_target_dependencies(node_movement rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp nikita_interfaces nlohmann_json)

include_directories(
  include/nikita_movement/
)

install(TARGETS
  node_movement
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
