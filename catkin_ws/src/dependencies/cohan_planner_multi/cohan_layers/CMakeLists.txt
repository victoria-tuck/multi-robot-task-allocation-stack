cmake_minimum_required(VERSION 2.8.3)
project(cohan_layers)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  cohan_msgs
  pluginlib
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
  cfg/AgentLayer.cfg
  cfg/AgentVLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  cohan_layers
  CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs cohan_msgs pluginlib roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cohan_layers
  src/agent_layer.cpp
  src/static_agent_layer.cpp
  src/agent_visibility_layer.cpp
)


add_dependencies(cohan_layers  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cohan_layers  ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS cohan_layers
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
