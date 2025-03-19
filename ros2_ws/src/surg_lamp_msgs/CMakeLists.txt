cmake_minimum_required(VERSION 3.8)
project(surg_lamp_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define message files before generating interfaces
set(MSG_FILES
  "msg/UserCommand.msg"
  "msg/LampTargetPose.msg"
  "msg/LampJointCommands.msg"
  "msg/TrackingData.msg"
  "msg/LampStatus.msg"
)

# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs
)

# Enable testing if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(geometry_msgs rosidl_runtime_cmake rosidl_runtime_py)

ament_package()
