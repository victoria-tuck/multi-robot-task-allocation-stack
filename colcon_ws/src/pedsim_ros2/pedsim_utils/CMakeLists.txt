cmake_minimum_required(VERSION 3.5)
project(pedsim_utils)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)

find_package(geometry_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)

# set(PACKAGE_DEPS
#  geometry_msgs
#  pedsim_msgs
#  rclcpp
# #  rclpy
# )

find_package(ament_cmake)# REQUIRED COMPONENTS ${PACKAGE_DEPS})
# find_package(${PACKAGE_DEPS})

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
# include_directories(
#   ${PROJECT_SOURCE_DIR}/include/pedsim_utils/
# )
include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include/pedsim_utils/
)
# set(HEADER_FILES include/geometry.hpp)
###########
## Build ##
###########
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
# include_directories(include)

# set(LIBRARY_NAME ${PROJECT_NAME})
# set(HEADER_FILES include/geometry.hpp)
add_library(pedsim_utils
  src/${PROJECT_NAME}/geometry.cpp
  src/${PROJECT_NAME}/pedsim_utils.cpp
  # ${HEADER_FILES}
)
ament_target_dependencies(pedsim_utils rclcpp geometry_msgs pedsim_msgs )#${PACKAGE_DEPS})
target_include_directories(pedsim_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_export_libraries(pedsim_utils)
# ament_export_include_directories(include)
ament_export_targets(pedsim_utils HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/pedsim_utils
  DESTINATION include
)

install(
  TARGETS pedsim_utils
  EXPORT pedsim_utils
  LIBRARY DESTINATION lib#/${PROJECT_NAME}
  ARCHIVE DESTINATION lib#/${PROJECT_NAME}
  RUNTIME DESTINATION bin#/${PROJECT_NAME} # do not do this if want to be public
  INCLUDES DESTINATION include#/${PROJECT_NAME}
)
ament_package()

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
