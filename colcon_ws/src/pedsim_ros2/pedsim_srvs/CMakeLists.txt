cmake_minimum_required(VERSION 2.8.3)
project(pedsim_srvs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)

# the services
set( srv_files
  "srv/SetAgentState.srv"
  "srv/GetAgentState.srv"
  "srv/SetAllAgentsState.srv"
  "srv/GetAllAgentsState.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()