cmake_minimum_required(VERSION 3.16)
project(nikita_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nikita_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Exportes for downstream packages
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

# Unit tests
if(BUILD_TESTING)
  ament_add_gtest(test_callback_timer 
    test/test_callback_timer.cpp
  )
  if(TARGET test_callback_timer)
    ament_target_dependencies(test_callback_timer rclcpp ament_index_cpp)
    target_include_directories(test_callback_timer PRIVATE include)
    target_link_libraries(test_callback_timer nikita_utils)
  endif()

  ament_add_gtest(test_simple_timer 
    test/test_simpletimer.cpp
  )
  if(TARGET test_simple_timer)
    ament_target_dependencies(test_simple_timer rclcpp ament_index_cpp)
    target_include_directories(test_simple_timer PRIVATE include)
    target_link_libraries(test_simple_timer nikita_utils)
  endif()

  ament_add_gtest(test_pid_controller
    test/test_pid_controller.cpp
  )
  if(TARGET test_pid_controller)
    ament_target_dependencies(test_pid_controller rclcpp ament_index_cpp)
    target_include_directories(test_pid_controller PRIVATE include)
    target_link_libraries(test_pid_controller nikita_utils)
  endif()

  ament_add_gtest(test_filters
    test/test_filters.cpp
  )
  if(TARGET test_filters)
    ament_target_dependencies(test_filters rclcpp ament_index_cpp geometry_msgs nikita_interfaces)
    target_include_directories(test_filters PRIVATE include)
    target_link_libraries(test_filters nikita_utils)
  endif()
endif()


ament_package()
