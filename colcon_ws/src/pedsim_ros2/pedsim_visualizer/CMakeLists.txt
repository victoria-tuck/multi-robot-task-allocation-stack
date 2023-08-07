cmake_minimum_required(VERSION 3.5)
project(pedsim_visualizer)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)
find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(geometry_msgs)
find_package(pedsim_msgs)
find_package(pedsim_utils)
find_package(rclcpp)
find_package(rclpy)
find_package(std_msgs)
find_package(visualization_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# generate_dynamic_reconfigure_options(
#   config/PedsimVisualizer.cfg
# )

###################################
## catkin specific configuration ##
###################################
# catkin_package(
#   CATKIN_DEPENDS ${PACKAGE_DEPS}
#   INCLUDE_DIRS include
# )

###########
## Build ##
###########
include_directories(include)

set(EXECUTABLE_NAME ${PROJECT_NAME}_node)
add_executable(${EXECUTABLE_NAME}
  src/sim_visualizer_node.cpp
  src/sim_visualizer.cpp
)

ament_target_dependencies(${EXECUTABLE_NAME} geometry_msgs pedsim_msgs pedsim_utils rclcpp std_msgs visualization_msgs)

target_link_libraries(${EXECUTABLE_NAME} pedsim_utils::pedsim_utils)

#############
## Install ##
#############

install(
  DIRECTORY include/pedsim_visualizer
  DESTINATION include
)

## Mark executables and/or libraries for installation
install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_visualizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
