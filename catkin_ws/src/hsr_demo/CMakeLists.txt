cmake_minimum_required(VERSION 2.8.3)
project(hsr_demo)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  tf
  tf2
  #NEW below
  dynamic_reconfigure
  message_generation
)

#generate_dynamic_reconfigure_options(
#  #cfg/gains.cfg
#  )

#add_message_files(
#  FILES
#  Inputs.msg
#  )


#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  tf
  tf2
)

find_package(Eigen3)



include_directories(${catkin_INCLUDE_DIRS})

#add_executable(controller_node src/controller_node.cpp)
#add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

