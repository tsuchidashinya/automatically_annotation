cmake_minimum_required(VERSION 3.0.2)
project(semantic_annotation)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  denso_msgs
  message_filters
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)
# find_package(VTK REQUIRED)
# find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES semantic_annotation
#  CATKIN_DEPENDS cv_bridge denso_msgs message_filters message_generation pcl_ros roscpp sensor_msgs std_msgs tf tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(box_extract_3d src/box_extract_3d.cpp src/box_extract_3d_node.cpp)
target_link_libraries(box_extract_3d
  ${catkin_LIBRARIES}
)

add_executable(mesh_cloud_publisher src/mesh_sampler.cpp src/mesh_cloud_publisher.cpp src/mesh_cloud_publisher_node.cpp)
target_link_libraries(mesh_cloud_publisher
  ${catkin_LIBRARIES}
)

add_executable(nearest_point_search src/nearest_point_search.cpp src/nearest_point_search_node.cpp)
target_link_libraries(nearest_point_search
  ${catkin_LIBRARIES}
)

add_executable(visualize_pointcloud src/visualize_pointcloud.cpp)
target_link_libraries(visualize_pointcloud
  ${catkin_LIBRARIES}
)