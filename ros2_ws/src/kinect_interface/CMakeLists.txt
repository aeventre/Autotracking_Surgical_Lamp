cmake_minimum_required(VERSION 3.8)
project(kinect_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Directly specify include and link directories
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Declare the executable
add_executable(kinect2_node src/kinect2_node.cpp)

# Directly link to the library
target_link_libraries(kinect2_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /usr/local/lib/libfreenect2.so  # Direct path
  pthread
)

# Install the executable
install(TARGETS kinect2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
