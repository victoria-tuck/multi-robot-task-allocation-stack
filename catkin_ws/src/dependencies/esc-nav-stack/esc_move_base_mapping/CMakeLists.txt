cmake_minimum_required(VERSION 3.0.2)
project(esc_move_base_mapping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  pedsim_msgs
  grid_map_core
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_ros
  grid_map_rviz_plugin
  grid_map_sdf
  grid_map_visualization
)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} 
  # pedsim_msgs
  DEPENDS octomap PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIR}
)

add_executable(esc_move_base_mapper src/octomap_gridmap_pc.cpp)
target_link_libraries(esc_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

