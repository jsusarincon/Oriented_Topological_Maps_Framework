cmake_minimum_required(VERSION 2.8.3)
project(collision_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  gazebo_plugins
  gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

add_message_files(
  FILES
  collision.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp message_runtime
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories($GAZEBO_LIBRARY_DIRS})
include_directories($GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
         ${Boost_INCLUDE_DIR}
         ${GAZEBO_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME} src/ContactPlugin.cc)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ) 

install(DIRECTORY models/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

