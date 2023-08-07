cmake_minimum_required(VERSION 3.5)
project(social_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(ros2launch REQUIRED)

# this needed to tell colcon to place launch files in the share folder
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()