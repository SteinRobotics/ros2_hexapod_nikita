cmake_minimum_required(VERSION 3.8)
project(nikita_movement)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


## set sources
set(SOURCES
  src/requester/requester.cpp
  src/requester/kinematics.cpp
  src/requester/gaitcontroller.cpp
  src/requester/lowpassfilter.cpp
  src/requester/actionpackagesparser.cpp
  src/handler/servohandler.cpp
  src/handler/servo_controller.cpp
  src/handler/servo_protocol.cpp
)

add_library(nikita_movement_core
  ${SOURCES}
)
target_include_directories(nikita_movement_core PUBLIC include/nikita_movement include/nikita_movement/requester)
ament_target_dependencies(nikita_movement_core rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp nikita_interfaces)
target_link_libraries(nikita_movement_core yaml-cpp::yaml-cpp)

add_executable(node_movement
  src/node_movement.cpp
)
target_link_libraries(node_movement
  nikita_movement_core
)
ament_target_dependencies(node_movement rclcpp)

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

# Unit tests
if(BUILD_TESTING)
  ament_add_gtest(test_kinematics
    test/test_kinematics.cpp
  )
  if(TARGET test_kinematics)
    ament_target_dependencies(test_kinematics rclcpp nikita_interfaces)
    target_link_libraries(test_kinematics nikita_movement_core)
  endif()

  ament_add_gtest(test_actionpackagesparser
    test/test_actionpackagesparser.cpp
  )
  if(TARGET test_actionpackagesparser)
    ament_target_dependencies(test_actionpackagesparser rclcpp ament_index_cpp nikita_interfaces)
    target_link_libraries(test_actionpackagesparser nikita_movement_core)
  endif()

  ament_add_gtest(test_requester
    test/test_requester.cpp
  )
  if(TARGET test_requester)
    ament_target_dependencies(test_requester rclcpp nikita_interfaces ament_index_cpp)
    target_link_libraries(test_requester nikita_movement_core)
  endif()
endif()

ament_package()
