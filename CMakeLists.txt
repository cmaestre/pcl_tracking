cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_tracking)

find_package(PCL 1.7 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

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

add_message_files(
  FILES
  ObjectPosition.msg  
)

add_service_files(
  FILES
  ObjectCloud.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

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

add_executable (object_conversions src/object_conversions.cpp)
target_link_libraries (object_conversions ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Tests

add_executable (planar_segmentation test/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (cluster_euclid test/cluster_euclid.cpp)
target_link_libraries (cluster_euclid ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (example_extract_clusters_normals test/example_extract_clusters_normals.cpp)
target_link_libraries (example_extract_clusters_normals ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (cluster_extraction test/cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable (correspondence_grouping test/correspondence_grouping.cpp)
#target_link_libraries (correspondence_grouping ${catkin_LIBRARIES} ${PCL_LIBRARIES})
