cmake_minimum_required(VERSION 3.5)
project(rfid_antenna_plugin)

# Load Ament CMake and find necessary packages
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rfid_msgs REQUIRED)

# Add Gazebo and ROS 2 include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Build the plugin library
add_library(${PROJECT_NAME} SHARED src/rfid_antenna.cpp)

# Link the plugin library with Gazebo and ROS 2 libraries
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Declare dependencies for the plugin (Ament specific)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  gazebo
  visualization_msgs
  rfid_msgs
)

# Install the plugin library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create an Ament package
ament_package()

