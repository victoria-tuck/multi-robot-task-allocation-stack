cmake_minimum_required(VERSION 3.5)
project(pedsim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wunused)

#------------------ Configuration ------------------#
# option(SHALL_DEBUG "Enable debug features" OFF)
# option(SHALL_PROFILE "Enable the code profiling feature" OFF)
# option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)



# #debugging
# if(SHALL_DEBUG)
#   message("Debugging activated")
#   add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
# else(SHALL_DEBUG)
#   message("Debugging deactivated")
#   add_definitions(-Os)
# endif(SHALL_DEBUG)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
# include_directories(
#   ${PROJECT_SOURCE_DIR}/include/pedsim/
# )
include_directories(include)
set(SOURCES
  src/ped_agent.cpp
  src/ped_angle.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)
add_library(pedsim ${SOURCES})
target_link_libraries(pedsim
  ${BOOST_LIBRARIES}
)
target_include_directories(pedsim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_export_libraries(pedsim)
ament_export_include_directories(include)
ament_export_targets(pedsim HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/pedsim
  DESTINATION include
)
install(
  TARGETS pedsim
  EXPORT pedsim
  LIBRARY DESTINATION lib#/${PROJECT_NAME}
  ARCHIVE DESTINATION lib#/${PROJECT_NAME}
  RUNTIME DESTINATION bin#/${PROJECT_NAME}
  INCLUDES DESTINATION include#/${PROJECT_NAME}
)
ament_package()