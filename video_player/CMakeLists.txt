cmake_minimum_required(VERSION 3.8)
project(video_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(video_player_node 
  src/video_player_node.cpp
  src/video_player.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(video_player_node ${OpenCV_LIBRARIES})

ament_target_dependencies(video_player_node 
  rclcpp 
  sensor_msgs 
  OpenCV 
  camera_interfaces
  cv_bridge
)

# Install executable
install(TARGETS video_player_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config files if they exist
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py" PATTERN "*.yaml" PATTERN "*.xml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
