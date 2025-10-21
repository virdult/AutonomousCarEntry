cmake_minimum_required(VERSION 3.8)
project(teleop_cpp)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# --- Executable: Teleop Key Controller ---
add_executable(virdult_teleop_key src/virdult_teleop_key.cpp)
ament_target_dependencies(virdult_teleop_key
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
)

# --- Executable: Path Publisher ---
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  turtlesim
)

# --- Executable: Stanley Controller ---
add_executable(stanley_controller src/stanley_controller.cpp)
ament_target_dependencies(stanley_controller
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  turtlesim
)

# --- Installation ---
install(TARGETS
  virdult_teleop_key
  path_publisher
  stanley_controller
  DESTINATION lib/${PROJECT_NAME}
)

# --- Package Declaration ---
ament_package()
