cmake_minimum_required(VERSION 3.8)
project(tello_uwb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17 for structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlink_parser REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(tello_uwb_py)

# Generate Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DronePosition.msg"
  "msg/DronePositionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Add uwb_republisher as a component library
add_library(uwb_republisher_component SHARED
  src/uwb_republisher.cpp
)

target_include_directories(uwb_republisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uwb_republisher_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nlink_parser
)

rclcpp_components_register_nodes(uwb_republisher_component
  "tello_uwb::UwbRepublisher"
)

# Add standalone executable
add_executable(uwb_republisher src/uwb_republisher.cpp)
target_include_directories(uwb_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uwb_republisher
  rclcpp
  rclcpp_components
  geometry_msgs
  nlink_parser
)

rosidl_get_typesupport_target(cpp_typesupport_target_republisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(uwb_republisher "${cpp_typesupport_target_republisher}")
target_link_libraries(uwb_republisher_component "${cpp_typesupport_target_republisher}")

# Install targets
install(TARGETS 
  uwb_republisher
  uwb_republisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python Executable
install(PROGRAMS
  tello_uwb_py/uwb_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
