cmake_minimum_required(VERSION 3.5)
project(pedsim_sensors)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)

# set(PACKAGE_DEPS
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pedsim_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# )

###########
## Build ##
###########

include_directories(include)

# People point cloud sensor.
# set(PEOPLE_PCD_EXEC_NAME pedsim_people_sensor)
# add_executable(${PEOPLE_PCD_EXEC_NAME} src/pedsim_sensors/people_point_cloud.cpp)
# ament_target_dependencies(${PEOPLE_PCD_EXEC_NAME} geometry_msgs nav_msgs pedsim_msgs rclcpp sensor_msgs pedsim_utils tf2 tf2_ros)

# Obstacle point cloud sensor.
set(OBSTACLE_PCD_EXEC_NAME pedsim_obstacle_sensor)
# add_executable(${OBSTACLE_PCD_EXEC_NAME} src/pedsim_sensors/obstacle_point_cloud.cpp)
# ament_target_dependencies(${OBSTACLE_PCD_EXEC_NAME} geometry_msgs nav_msgs pedsim_msgs rclcpp sensor_msgs pedsim_utils tf2 tf2_ros)
# target_link_libraries( ${OBSTACLE_PCD_EXEC_NAME} pedsim_utils::pedsim_utils  )

# # Point cloud sensor.
set(OCCLUSION_PCD_EXEC_NAME pedsim_occlusion_sensor)
add_executable(${OCCLUSION_PCD_EXEC_NAME} src/pedsim_sensors/occlusion_point_cloud.cpp)
ament_target_dependencies(${OCCLUSION_PCD_EXEC_NAME} geometry_msgs nav_msgs pedsim_msgs rclcpp sensor_msgs pedsim_utils tf2 tf2_ros)

#############
## Install ##
#############
install(TARGETS
    # ${PEOPLE_PCD_EXEC_NAME}
    # ${OBSTACLE_PCD_EXEC_NAME}
    ${OCCLUSION_PCD_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})
  # ament_package()
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_sensors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
