cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

include_directories(include)
include_directories(SYSTEM
  /usr/local/include  #to find lightsfm
  ${Boost_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  # ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

# gazebo_ros_diff_drive
# add_library(GazeboRosDiffDrivePlugin SHARED src/gazebo_ros_diff_drive.cpp
# )
# target_include_directories(GazeboRosDiffDrivePlugin PUBLIC include)
# ament_target_dependencies(GazeboRosDiffDrivePlugin
#   "gazebo_dev"
#   "gazebo_ros"
#   "geometry_msgs"
#   "nav_msgs"
#   "rclcpp"
#   "tf2" 1
#   "tf2_geometry_msgs"
#   "tf2_ros"
# )
# ament_export_libraries(GazeboRosDiffDrivePlugin)
# target_link_libraries(GazeboRosDiffDrivePlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES
# install(TARGETS GazeboRosDiffDrivePlugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


add_library(PedestrianSFMPlugin SHARED src/PedestrianSFMPlugin.cpp)
target_include_directories(PedestrianSFMPlugin PUBLIC include)
ament_target_dependencies(PedestrianSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
)
ament_export_libraries(PedestrianSFMPlugin)
target_link_libraries(PedestrianSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS PedestrianSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(ActorStatePublisherPlugin SHARED
  src/ActorStatePublisherPlugin.cpp
)
target_include_directories(ActorStatePublisherPlugin PUBLIC include)
ament_target_dependencies(ActorStatePublisherPlugin
  "gazebo_ros"
  "gazebo_dev"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(ActorStatePublisherPlugin)
install(TARGETS ActorStatePublisherPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(ActorVelocitySubscriberPlugin SHARED
  src/ActorVelocitySubscriberPlugin.cpp
)
target_include_directories(ActorVelocitySubscriberPlugin PUBLIC include)
ament_target_dependencies(ActorVelocitySubscriberPlugin
  "gazebo_ros"
  "gazebo_dev"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(ActorVelocitySubscriberPlugin)
install(TARGETS ActorVelocitySubscriberPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_library(ModelStatePublisherPlugin SHARED
  src/ModelStatePublisherPlugin.cpp
)
target_include_directories(ModelStatePublisherPlugin PUBLIC include)
ament_target_dependencies(ModelStatePublisherPlugin
  "gazebo_ros"
  "gazebo_dev"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(ModelStatePublisherPlugin)
install(TARGETS ModelStatePublisherPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY media/models
   DESTINATION share/${PROJECT_NAME}/
)



ament_package()
