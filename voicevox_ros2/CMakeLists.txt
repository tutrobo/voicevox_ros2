cmake_minimum_required(VERSION 3.8)
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

# 依存関係
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(voicevox_core_vendor REQUIRED)
find_package(voicevox_ros2_msgs REQUIRED)

# Composable Nodeを作成
add_library(${PROJECT_NAME}_component SHARED src/voicevox_ros2_component.cpp)
target_include_directories(${PROJECT_NAME}_component PRIVATE include)
target_link_libraries(${PROJECT_NAME}_component
  SDL2
  SDL2_mixer
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  voicevox_core_vendor
  voicevox_ros2_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "tutrobo::VoicevoxRos2"
  EXECUTABLE ${PROJECT_NAME}
)

# インストール
ament_export_targets(export_${PROJECT_NAME}_component HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  voicevox_core_vendor
  voicevox_ros2_msgs
)

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_package()