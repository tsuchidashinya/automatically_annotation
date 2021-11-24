cmake_minimum_required(VERSION 3.0.2)
project(yolo_annotation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  denso_msgs
  denso_srvs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_annotation
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport message_filters pcl_ros roscpp senser_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(yolo_annotation src/yolo_annotation.cpp)
target_link_libraries(yolo_annotation
  ${catkin_LIBRARIES}
)

add_executable(yolo_annotation_kai src/yolo_annotation_kai.cpp)
target_link_libraries(yolo_annotation_kai
  ${catkin_LIBRARIES}
)

add_executable(yolo_annotation_kai_all_HV8 src/yolo_annotation_kai_all_HV8.cpp)
target_link_libraries(yolo_annotation_kai_all_HV8
  ${catkin_LIBRARIES}
)

add_executable(message_casher src/message_casher.cpp)
target_link_libraries(message_casher
  ${catkin_LIBRARIES}
)

add_executable(sensor_move_try src/sensor_move_try.cpp)
target_link_libraries(sensor_move_try
  ${catkin_LIBRARIES}
)

add_executable(sensor_service_move src/sensor_service_move.cpp)
target_link_libraries(sensor_service_move
  ${catkin_LIBRARIES}
)

add_executable(sensor_angle_move src/sensor_angle_move.cpp)
target_link_libraries(sensor_angle_move
  ${catkin_LIBRARIES}
)