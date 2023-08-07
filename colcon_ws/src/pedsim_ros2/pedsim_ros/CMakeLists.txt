# cmake_minimum_required(VERSION 2.8.3)
# project(pedsim_ros)
# find_package(catkin REQUIRED)
# catkin_metapackage()


cmake_minimum_required(VERSION 3.5)
project(pedsim_ros)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)
find_package(ament_cmake REQUIRED)
ament_package()