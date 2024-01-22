cmake_minimum_required(VERSION 3.8)
project(norlab_controllers_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "Release")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  "msg/DirectionalPath.msg"
  "msg/PathSequence.msg"
  "msg/FollowerOptions.msg"
  "srv/ExportData.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
        ${norlab_controller_msgs_INCLUDE_DIRS}
)

ament_package()
