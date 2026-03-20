cmake_minimum_required(VERSION 3.5)
project(controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrackSineForce.action"
  "action/TrackSineLength.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(controller_pid SHARED
  src/pid.cpp
)

target_include_directories(controller_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cylinder_controller
  src/cylinder_controller.cpp
)

target_link_libraries(cylinder_controller
  controller_pid
  ${cpp_typesupport_target}
)

ament_target_dependencies(cylinder_controller
  rclcpp
  rclcpp_action
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS controller_pid
  EXPORT export_controller_pid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS cylinder_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(controller_pid)
ament_export_targets(export_controller_pid HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
