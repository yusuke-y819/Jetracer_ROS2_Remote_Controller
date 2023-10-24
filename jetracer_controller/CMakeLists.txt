cmake_minimum_required(VERSION 3.8)
project(jetracer_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(logicool_handle src/logicool_handle.cpp src/controller.cpp)
ament_target_dependencies(logicool_handle rclcpp control_msgs geometry_msgs)

add_executable(velocity_selector src/velocity_selector.cpp)
ament_target_dependencies(velocity_selector rclcpp control_msgs)

install(TARGETS
  logicool_handle
  velocity_selector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
