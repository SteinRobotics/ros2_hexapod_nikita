cmake_minimum_required(VERSION 3.8)
project(nikita_brain)

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
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(GTest REQUIRED)

## set sources
set(SOURCES
src/requester/coordinator.cpp
src/action/action_planner.cpp
src/requester/recognition.cpp
src/requester/error_management.cpp
src/requester/text_interpreter.cpp
src/handler/system.cpp
src/handler/communication.cpp
src/handler/movement.cpp
)
add_library(nikita_brain_lib ${SOURCES})
target_include_directories(nikita_brain_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nikita_brain_lib rclcpp ament_index_cpp nikita_interfaces nlohmann_json)

add_executable(node_brain
  src/node_brain.cpp
)
target_link_libraries(node_brain nikita_brain_lib)
target_include_directories(node_brain PRIVATE include)
ament_target_dependencies(node_brain rclcpp std_msgs geometry_msgs std_srvs ament_index_cpp nikita_interfaces nlohmann_json)

include_directories(
  include/nikita_brain/
)

install(TARGETS
  node_brain
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# Unit tests
if(BUILD_TESTING)
  ament_add_gtest(test_text_interpreter
    test/test_text_interpreter.cpp
  )
  if(TARGET test_text_interpreter)
    ament_target_dependencies(test_text_interpreter rclcpp ament_index_cpp nikita_interfaces nlohmann_json)
    target_include_directories(test_text_interpreter PRIVATE include)
    target_link_libraries(test_text_interpreter nikita_brain_lib)
  endif()
  # Fallback: also create an explicit test executable and register it with CTest
  add_executable(test_text_interpreter_exec test/test_text_interpreter.cpp)
  target_include_directories(test_text_interpreter_exec PRIVATE include)
  ament_target_dependencies(test_text_interpreter_exec rclcpp ament_index_cpp nikita_interfaces nlohmann_json)
  target_link_libraries(test_text_interpreter_exec nikita_brain_lib)
  # Link gtest if available
  find_package(GTest QUIET)
  if(GTest_FOUND)
    target_link_libraries(test_text_interpreter_exec GTest::GTest GTest::Main)
  endif()
  add_test(NAME test_text_interpreter_exec COMMAND test_text_interpreter_exec)
endif()
