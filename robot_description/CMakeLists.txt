cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ament_cmake first - CRITICAL
find_package(ament_cmake REQUIRED)

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# For message generation
find_package(rosidl_default_generators REQUIRED)

# 1. Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# --- بخش گره‌های C++ ---

# A* Planner node
add_executable(astar_planner_node scripts/astar_planner.cpp)
ament_target_dependencies(astar_planner_node
  rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# PID Path Follower node
add_executable(pid_path_follower_node scripts/pid_path_follower.cpp)
ament_target_dependencies(pid_path_follower_node
  rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# LaserScan frame_id converter node
add_executable(frame_id_converter_node scripts/frame_id_converter.cpp)
ament_target_dependencies(frame_id_converter_node
  rclcpp sensor_msgs
)

include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(ekf_diff_imu_node scripts/ekf_diff_imu.cpp)
ament_target_dependencies(ekf_diff_imu_node
  rclcpp 
  sensor_msgs 
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(ekf_diff_imu_node Eigen3::Eigen)

# Motor Command node (جدید - موجود در لیست فایل‌های شما)
add_executable(motor_command_node scripts/motor_command.cpp)
ament_target_dependencies(motor_command_node
  rclcpp geometry_msgs
)

# Link service interfaces to nodes that use them
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(astar_planner_node "${cpp_typesupport_target}")
# اگر نودهای دیگر هم از این سرویس استفاده می‌کنند، خط بالا را برای آن‌ها تکرار کنید.

# --- بخش نصب (Install) ---

# Install C++ executables
install(TARGETS
  astar_planner_node
  pid_path_follower_node
  frame_id_converter_node
  ekf_diff_imu_node
  motor_command_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install Python scripts (بسیار مهم برای فایل‌های RL و محیط Gazebo)
install(PROGRAMS
  scripts/evaluate_rl_agent.py
  scripts/gym_gazebo_env.py
  scripts/rl_agent.py
  scripts/test_env.py
  scripts/train_rl_agent.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install non-code resources
install(
  DIRECTORY
    launch
    rviz
    config
    world   # اضافه شد چون در فایل لانچ به آن اشاره شده بود
    maps    # اضافه شد برای Map Server
    src     # اگر فایل‌های URDF اینجا هستند
    srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()