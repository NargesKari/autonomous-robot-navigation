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


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)


# A* node
add_executable(astar_node scripts/astar.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(astar_node "${cpp_typesupport_target}")

ament_target_dependencies(astar_node
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs
)

# PID Controller node
add_executable(pid_controller_node scripts/pid_controller.cpp)
ament_target_dependencies(pid_controller_node
  rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# MPC Controller node
add_executable(mpc_controller_node scripts/mpc_controller.cpp)
ament_target_dependencies(mpc_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs  
)
target_link_libraries(mpc_controller_node Eigen3::Eigen)


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

add_executable(motor_command_node scripts/motor_command.cpp)
ament_target_dependencies(motor_command_node
  rclcpp geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(astar_node "${cpp_typesupport_target}")

install(TARGETS
  astar_node
  pid_controller_node
  mpc_controller_node
  frame_id_converter_node
  ekf_diff_imu_node
  motor_command_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY
    launch
    rviz
    config
    world  
    maps    
    src    
    srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()