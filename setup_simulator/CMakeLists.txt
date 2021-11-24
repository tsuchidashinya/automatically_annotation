cmake_minimum_required(VERSION 3.0.2)
project(setup_simulator)


# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  tf
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES setup_simulator
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs pcl_ros roscpp tf tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(tf_publisher  src/tf_publisher.cpp)
target_link_libraries(tf_publisher
  ${catkin_LIBRARIES}
)

add_executable(cloud_planar_segmenter src/cloud_planar_segmenter.cpp)
target_link_libraries(cloud_planar_segmenter
  ${catkin_LIBRARIES}
  ${pcl_LIBRARIES}
)