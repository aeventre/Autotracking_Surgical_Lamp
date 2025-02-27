cmake_minimum_required(VERSION 3.8)
project(kinect_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include libfreenect2
include_directories(/usr/local/include/libfreenect2)  # Adjust path if necessary
link_directories(/usr/local/lib)  # Adjust path if necessary

# Declare the executable and link to libraries
add_executable(kinect2_node src/kinect2_node.cpp)

target_link_libraries(kinect2_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libfreenect2
  pthread
)

# Install the executable
install(TARGETS kinect2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
