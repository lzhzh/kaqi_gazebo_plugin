cmake_minimum_required(VERSION 2.8.3)
project(kaqi_gazebo_plugin)

add_definitions(-std=c++11)

add_definitions(-DROS_ASSERT_ENABLED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  std_srvs
  tf
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_kaqi_base_plugin
  CATKIN_DEPENDS geometry_msgs std_srvs tf
  DEPENDS gazebo
)

include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${Eigen_INCLUDE_DIRS})

add_library(gazebo_kaqi_base_plugin src/gazebo_kaqi_base_plugin.cpp)
target_link_libraries(gazebo_kaqi_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_kaqi_base_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    gazebo_kaqi_base_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
