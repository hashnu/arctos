cmake_minimum_required(VERSION 3.8)
project(arctos)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(canary REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/libsockcanpp)

add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/${PROJECT_NAME}_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  pluginlib
  rcpputils
  hardware_interface
  realtime_tools
  trajectory_msgs
  controller_manager
  canary
  Boost
  )

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY description/meshes description/rviz description/urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  arctos/scripts/set_joints_zero_positions.py
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib  rcpputils hardware_interface controller_interface realtime_tools trajectory_msgs controller_manager)
ament_package()
