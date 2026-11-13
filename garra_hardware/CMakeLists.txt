cmake_minimum_required(VERSION 3.16)
project(garra_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_library(garra_hardware SHARED src/garra_servo_system.cpp)
target_compile_features(garra_hardware PUBLIC cxx_std_17)
target_compile_options(garra_hardware PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(garra_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(garra_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

pluginlib_export_plugin_description_file(hardware_interface garra_hardware.xml)

install(TARGETS garra_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_package()
