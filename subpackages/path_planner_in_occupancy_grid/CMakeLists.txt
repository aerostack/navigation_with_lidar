cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME path_planner_in_occupancy_grid)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

set(PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_DIR
  src/source
)

set(PATH_PLANNER_IN_OCCUPANCY_GRID_INCLUDE_DIR
  src/include
)

set(PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_FILES
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_DIR}/path_planner_in_occupancy_grid.cpp
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_DIR}/path_planner_in_occupancy_grid_main.cpp
)

set(PATH_PLANNER_IN_OCCUPANCY_GRID_HEADER_FILES
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_INCLUDE_DIR}/path_planner_in_occupancy_grid.h
)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  robot_process
  droneMsgsROS
  aerostack_msgs
)

catkin_package(
  INCLUDE_DIRS ${PATH_PLANNER_IN_OCCUPANCY_GRID_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  robot_process
  droneMsgsROS
  aerostack_msgs
)

include_directories(
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_INCLUDE_DIR}
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_DIR}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(
  path_planner_in_occupancy_grid_lib
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_FILES}
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_HEADER_FILES}
)

add_dependencies(path_planner_in_occupancy_grid_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_in_occupancy_grid_lib ${catkin_LIBRARIES})

add_executable(
  path_planner_in_occupancy_grid
  ${PATH_PLANNER_IN_OCCUPANCY_GRID_SOURCE_DIR}/path_planner_in_occupancy_grid_main.cpp
)

add_dependencies(path_planner_in_occupancy_grid ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_in_occupancy_grid path_planner_in_occupancy_grid_lib)
target_link_libraries(path_planner_in_occupancy_grid ${catkin_LIBRARIES})
 