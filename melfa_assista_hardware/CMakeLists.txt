cmake_minimum_required(VERSION 3.8)
project(melfa_assista_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14) # Hier können Sie auch 14 oder 17 verwenden, je nach Ihren Anforderungen
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/melfa_assista_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)


target_compile_features(melfa_assista_hardware PUBLIC cxx_std_17)
target_include_directories(melfa_assista_hardware PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/melfa_assista_hardware>
)
ament_target_dependencies(
  melfa_assista_hardware PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

include_directories(
        include
)

pluginlib_export_plugin_description_file(hardware_interface melfa_assista_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
        DIRECTORY description/ros2_control description/urdf description/meshes description/config
        DESTINATION share/melfa_assista_hardware
)

install(TARGETS melfa_assista_hardware
        EXPORT export-melfa_assista_hardware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY bringup/launch 
        DESTINATION share/melfa_assista_hardware
)


ament_export_targets(export-melfa_assista_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()