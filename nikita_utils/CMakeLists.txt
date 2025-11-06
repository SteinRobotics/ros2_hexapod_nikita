cmake_minimum_required(VERSION 3.16)
project(nikita_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
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
  ament_add_gtest(test_simple_timer 
    test/test_simpletimer.cpp
  )
  if(TARGET test_simple_timer)
    ament_target_dependencies(test_simple_timer rclcpp ament_index_cpp)
    target_include_directories(test_simple_timer PRIVATE include)
    target_link_libraries(test_simple_timer nikita_utils)
  endif()
endif()


ament_package()
