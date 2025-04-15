cmake_minimum_required(VERSION 3.8)
project(goal_pose_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_common REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Executable and linking
add_executable(goal_pose_executor src/goal_pose_executor.cpp)
ament_target_dependencies(goal_pose_executor
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_common
  tf2_geometry_msgs
)

install(TARGETS goal_pose_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
