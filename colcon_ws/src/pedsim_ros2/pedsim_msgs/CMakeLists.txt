cmake_minimum_required(VERSION 3.5)
project(pedsim_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
# set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
# file(GLOB PEDSIM_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

## Generate messages in the 'msg' folder
set (msg_files
  "msg/AgentState.msg"
  "msg/AgentStates.msg"
  "msg/AgentGroup.msg"
  "msg/AgentGroups.msg"
  "msg/AgentForce.msg"
  "msg/LineObstacle.msg"
  "msg/LineObstacles.msg"
  "msg/TrackedPerson.msg"
  "msg/TrackedPersons.msg"
  "msg/TrackedGroup.msg"
  "msg/TrackedGroups.msg"
  "msg/SocialRelation.msg"
  "msg/SocialRelations.msg"
  "msg/SocialActivity.msg"
  "msg/SocialActivities.msg"
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
