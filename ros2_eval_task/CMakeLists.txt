cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ==============================================================
# Dependencies
# ==============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

# ==============================================================
# Include directories (for headers in include/)
# ==============================================================
include_directories(
  include
)

# ==============================================================
# Build Shared Library (Gazebo Utils Client)
# ==============================================================
add_library(gazebo_utils_client SHARED
  src/gazebo_utils_client.cpp
)

ament_target_dependencies(gazebo_utils_client
  rclcpp
  gazebo_msgs
  geometry_msgs
  visualization_msgs
)

# ==============================================================
# Build Executables (ROS2 Nodes)
# ==============================================================
add_executable(model_spawner_node
  src/model_spawner_node.cpp
)

ament_target_dependencies(model_spawner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  ament_index_cpp
  visualization_msgs
  cv_bridge
)

# Link against the shared library and OpenCV
target_link_libraries(model_spawner_node
  gazebo_utils_client
  ${OpenCV_LIBS}
)

# ==============================================================
# Installation (Binaries + Library)
# ==============================================================
install(TARGETS
  model_spawner_node
  gazebo_utils_client
  EXPORT ros2_eval_task_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# export targets so sourced workspace can find library
ament_export_targets(ros2_eval_task_targets HAS_LIBRARY_TARGET)

# ==============================================================
# Install Shared Resources (launch, rviz, worlds, models, etc.)
# ==============================================================
install(DIRECTORY
  launch
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================
# Finalize Package
# ==============================================================
ament_package()
