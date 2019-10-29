cmake_minimum_required(VERSION 2.8.3)
project(grid_mapper)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
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
  laser_geometry
)


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
#add_compile_options(-std=c++98)
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(grid_mapper  src/grid_mapper.cpp)
target_link_libraries(grid_mapper ${catkin_LIBRARIES})
target_link_libraries(grid_mapper ${OpenCV_LIBS} ${BULLET_LIBRARIES})
add_dependencies(grid_mapper ${PROJECT_NAME}_generate_messages_cpp)
