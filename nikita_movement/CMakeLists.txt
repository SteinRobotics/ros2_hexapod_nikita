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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nikita_interfaces REQUIRED)
find_package(nikita_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(magic_enum CONFIG REQUIRED)


## set sources
set(SOURCES
  src/requester/requester.cpp
  src/requester/kinematics.cpp
  src/requester/gaitcontroller.cpp
  src/requester/actionpackagesparser.cpp
  src/requester/gait_tripod.cpp
  src/requester/gait_ripple.cpp
  src/requester/gait_bodyroll.cpp
  src/requester/gait_legwave.cpp
  src/requester/gait_waiting.cpp
  src/handler/servohandler.cpp
  src/handler/servo_controller.cpp
  src/handler/servo_protocol.cpp
)

add_library(nikita_movement_core
  ${SOURCES}
)
target_include_directories(nikita_movement_core PUBLIC include/nikita_movement include/nikita_movement/requester)
ament_target_dependencies(nikita_movement_core rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp nikita_utils nikita_interfaces)
ament_target_dependencies(nikita_movement_core sensor_msgs)
target_link_libraries(nikita_movement_core yaml-cpp::yaml-cpp magic_enum::magic_enum)

add_executable(node_movement
  src/node_movement.cpp
)
target_link_libraries(node_movement
  nikita_movement_core
)
ament_target_dependencies(node_movement rclcpp nikita_utils)

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
    ament_target_dependencies(test_kinematics rclcpp ament_index_cpp nikita_interfaces)
    target_include_directories(test_kinematics PRIVATE include)
    target_link_libraries(test_kinematics nikita_movement_core)
  endif()

  ament_add_gtest(test_new_kinematics
    test/test_new_kinematics.cpp
  )
  if(TARGET test_new_kinematics)
    ament_target_dependencies(test_new_kinematics rclcpp ament_index_cpp nikita_interfaces)
    target_include_directories(test_new_kinematics PRIVATE include)
    target_link_libraries(test_new_kinematics nikita_movement_core)
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

  ament_add_gtest(test_gait_bodyroll
    test/test_gait_bodyroll.cpp
  )
  if(TARGET test_gait_bodyroll)
    ament_target_dependencies(test_gait_bodyroll rclcpp nikita_interfaces ament_index_cpp)
    target_link_libraries(test_gait_bodyroll nikita_movement_core)
  endif()

  ament_add_gtest(test_complete_body_init
    test/test_complete_body_init.cpp
  )
  if(TARGET test_complete_body_init)
    ament_target_dependencies(test_complete_body_init rclcpp ament_index_cpp nikita_interfaces)
    target_include_directories(test_complete_body_init PRIVATE include)
    target_link_libraries(test_complete_body_init nikita_movement_core)
  endif()

  ament_add_gtest(test_moveBodyNew_publish
    test/test_moveBodyNew_publish.cpp
  )
  if(TARGET test_moveBodyNew_publish)
    ament_target_dependencies(test_moveBodyNew_publish rclcpp ament_index_cpp nikita_interfaces sensor_msgs)
    target_include_directories(test_moveBodyNew_publish PRIVATE include)
    target_link_libraries(test_moveBodyNew_publish nikita_movement_core)
  endif()

endif()

ament_package()
