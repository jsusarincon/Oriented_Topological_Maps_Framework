cmake_minimum_required(VERSION 2.8.3)
project(logical_camera_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  gazebo_plugins
  gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

#find_package (SDL2 REQUIRED)

add_message_files(
  FILES
  logicalImage.msg
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
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
		     ${Boost_INCLUDE_DIR}
		     ${GAZEBO_INCLUDE_DIRS}
#		     ${SDL2_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} src/logical_camera_plugin.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ) 

install(DIRECTORY models/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

#add_executable(keyboard src/keyboardcontrol.cpp)
#target_link_libraries(keyboard ${catkin_LIBRARIES})
