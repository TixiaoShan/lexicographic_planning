cmake_minimum_required(VERSION 2.8.3)
project(lexicographic_planning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -O3 -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  # pcl library
  pcl_ros
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pluginlib
  nav_core
)

find_package(OpenMP)
find_package(PCL REQUIRED QUIET)



catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS 
  geometry_msgs 
  std_msgs
  nav_msgs
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Server
add_executable(${PROJECT_NAME}_pathServer src/pathServer.cpp)
target_link_libraries(${PROJECT_NAME}_pathServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Planner
add_executable(${PROJECT_NAME}_pathPlanning src/pathPlanning.cpp)
target_link_libraries(${PROJECT_NAME}_pathPlanning ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Obstable Server
add_executable(${PROJECT_NAME}_obstacleServer src/obstacleServer.cpp)
target_link_libraries(${PROJECT_NAME}_obstacleServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Cloud transformer
add_executable(${PROJECT_NAME}_cloudRegister src/cloudRegister.cpp)
target_link_libraries(${PROJECT_NAME}_cloudRegister ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# move_base Planner Plugin
add_library(lex_planner_lib src/library/lex_planner.cpp)
target_link_libraries(lex_planner_lib ${catkin_LIBRARIES})