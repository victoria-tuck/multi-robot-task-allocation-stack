cmake_minimum_required(VERSION 3.5)
project(pedsim_utils)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)

set(PACKAGE_DEPS
  cmake_modules
  geometry_msgs
  pedsim_msgs
  rclcpp
  rclpy
)
find_package(ament_cmake REQUIRED COMPONENTS ${PACKAGE_DEPS})
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories(
  ${PROJECT_SOURCE_DIR}/include/pedsim_utils/
)

###########
## Build ##
###########
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
# include_directories(include)

# set(LIBRARY_NAME ${PROJECT_NAME})
add_library(pedsim_utils
  src/${PROJECT_NAME}/geometry.cpp
  src/${PROJECT_NAME}/pedsim_utils.cpp
)
ament_target_dependencies(pedsim_utils ${PACKAGE_DEPS})
target_include_directories(pedsim_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_export_libraries(pedsim_utils)
install(
  TARGETS pedsim_utils
  EXPORT export_pedsim_utils
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
