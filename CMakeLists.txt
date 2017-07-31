cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_tracking)

find_package(PCL 1.2 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  roscpp
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  pcl_ros
)

#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  LIBRARIES crustcrawler_cppkdl
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (create_model src/create_model.cpp)
target_link_libraries (create_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (auto_tracking src/auto_tracking.cpp)
target_link_libraries (auto_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES})