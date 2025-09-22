cmake_minimum_required(VERSION 3.8)
project(booster_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(booster_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/booster_topic_comm.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  booster_interface
  ament_index_cpp
)

# Link yaml-cpp
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Install libraries
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# No scripts to install currently

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  booster_interface
  ament_index_cpp
)

ament_package()