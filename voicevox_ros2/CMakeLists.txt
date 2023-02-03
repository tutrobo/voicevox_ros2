cmake_minimum_required(VERSION 3.17)
project(voicevox_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(voicevox_core_vendor REQUIRED)
find_package(voicevox_core REQUIRED)
find_package(voicevox_ros2_msgs REQUIRED)

# Composable Node
add_library(${PROJECT_NAME}_component SHARED
  src/voicevox_ros2_component.cpp
)
target_include_directories(${PROJECT_NAME}_component
  PRIVATE include
)
target_link_libraries(${PROJECT_NAME}_component voicevox_core::voicevox_core ao)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  voicevox_core_vendor
  voicevox_ros2_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN      "tutrobo::VoicevoxRos2"
  EXECUTABLE  "${PROJECT_NAME}"
)

# インストール
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
