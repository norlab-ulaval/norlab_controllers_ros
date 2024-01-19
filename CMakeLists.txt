cmake_minimum_required(VERSION 3.8)
project(norlab_controllers_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()