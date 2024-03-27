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
find_package(gazebo_msgs REQUIRED)
find_package(social_navigation_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)
include_directories(SYSTEM
  /usr/local/include #for lightsfm
)
# this needed to tell colcon to place launch files in the share folder
install(DIRECTORY launch worlds configs
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})


# add_executable(set_gazebo_state_node nodes/gazebo_set_state.cpp)
# ament_target_dependencies(set_gazebo_state_node rclcpp gazebo_msgs std_msgs)
# install( TARGETS
#   set_gazebo_state_node
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(get_human_states nodes/get_human_states.cpp)
ament_target_dependencies(get_human_states rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs visualization_msgs)
install( TARGETS
  get_human_states
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_goal nodes/test_goal.cpp)
ament_target_dependencies(test_goal rclcpp geometry_msgs )
install( TARGETS
  test_goal
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(get_human_closest_obstacle nodes/human_nearest_obstacle_from_map.cpp)
# ament_target_dependencies(get_human_closest_obstacle rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs sensor_msgs)
# install( TARGETS
#   get_human_closest_obstacle
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(robot_closest_obstacle nodes/robot_nearest_obstacle_from_map.cpp)
ament_target_dependencies(robot_closest_obstacle rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs sensor_msgs)
install( TARGETS
  robot_closest_obstacle
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(robot_closest_obstacle_sector nodes/robot_nearest_sector_obstacle_from_map.cpp)
ament_target_dependencies(robot_closest_obstacle_sector rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs sensor_msgs)
install( TARGETS
robot_closest_obstacle_sector
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(human_closest_obstacle_sector nodes/human_nearest_sector_obstacle_from_map.cpp)
ament_target_dependencies(human_closest_obstacle_sector rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs sensor_msgs)
install( TARGETS
human_closest_obstacle_sector
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(humans_sfm_node nodes/humans_sfm_node.cpp)
# ament_target_dependencies(humans_sfm_node rclcpp gazebo_msgs std_msgs geometry_msgs social_navigation_msgs nav_msgs)
# install( TARGETS
# humans_sfm_node
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()