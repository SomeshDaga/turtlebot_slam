cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_slam)

add_compile_options(-std=c++11)

# Find Boost libraries
find_package(Boost REQUIRED)

# Find the Eigen3 libraries
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  # Just change the name of the eigen variables
  # to the expectation of the catkin build system
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  set(Eigen3_LIBRARIES    ${EIGEN3_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Mapping.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    gazebo_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    rospy
    tf2
    tf2_ros
    visualization_msgs
  DEPENDS
    Boost
    Eigen3
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(mapping_node
  src/mapping.cpp
  src/mapping_node.cpp
)

add_dependencies(mapping_node
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(mapping_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(
  TARGETS mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)
