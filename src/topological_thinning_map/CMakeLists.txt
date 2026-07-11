cmake_minimum_required(VERSION 3.0.2)
project(topological_thinning_map)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cartographer_ros_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package( 
CATKIN_DEPENDS actionlib_msgs
)
find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thinning_purge src/thinning_purge.cpp)
target_link_libraries(thinning_purge ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})
