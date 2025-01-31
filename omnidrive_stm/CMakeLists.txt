cmake_minimum_required(VERSION 3.16)
project(omnidrive_stm LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  omnidrive_stm
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(omnidrive_stm PUBLIC cxx_std_17)
target_include_directories(omnidrive_stm PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/omnidrive_stm>
)
ament_target_dependencies(
  omnidrive_stm PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(omnidrive_stm PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "OMNIDRIVE_STM_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface omnidrive_stm.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/omnidrive_stm
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/omnidrive_stm
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/omnidrive_stm
)
install(TARGETS omnidrive_stm
  EXPORT export_omnidrive_stm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_omnidrive_stm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
