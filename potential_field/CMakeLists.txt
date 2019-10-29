cmake_minimum_required(VERSION 2.8.3)
project(potential_field)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs 
  message_filters
  sensor_msgs
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS
  message_runtime
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
)

###########
## Build ##
###########
add_compile_options(-std=c++98)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(potential_field src/potential_field.cpp)
target_link_libraries(potential_field ${catkin_LIBRARIES})
target_link_libraries(potential_field ${OpenCV_LIBRARIES} ${BULLET_LIBRARIES})
#add_dependencies(potential_field ${PROJECT_NAME}_generate_messages_cpp)


