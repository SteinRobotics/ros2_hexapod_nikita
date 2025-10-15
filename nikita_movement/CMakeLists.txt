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
  src/action/action_executor.cpp
  src/handler/servohandler.cpp
)

add_executable(node_movement
  src/node_movement.cpp
  ${SOURCES}
)
ament_target_dependencies(node_movement rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp nikita_interfaces)

target_link_libraries(node_movement
  yaml-cpp::yaml-cpp
)

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
    src/requester/kinematics.cpp
  )
  if(TARGET test_kinematics)
    ament_target_dependencies(test_kinematics rclcpp nikita_interfaces)
    target_include_directories(test_kinematics PRIVATE include/nikita_movement include/nikita_movement/requester)
  endif()
endif()

ament_package()
