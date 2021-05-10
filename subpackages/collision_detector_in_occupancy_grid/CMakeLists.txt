cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME collision_detector_in_occupancy_grid)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

set(COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_DIR
  src/source
)

set(COLLISION_DETECTOR_IN_OCCUPANCY_GRID_INCLUDE_DIR
  src/include
)

set(COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_FILES
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_DIR}/collision_detector_in_occupancy_grid.cpp
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_DIR}/collision_detector_in_occupancy_grid.cpp
)

set(COLLISION_DETECTOR_IN_OCCUPANCY_GRID_HEADER_FILES
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_INCLUDE_DIR}/collision_detector_in_occupancy_grid.h
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
  INCLUDE_DIRS ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  robot_process
  droneMsgsROS
  aerostack_msgs
)

include_directories(
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_INCLUDE_DIR}
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_DIR}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(
  collision_detector_in_occupancy_grid_lib
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_FILES}
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_HEADER_FILES}
)

add_dependencies(collision_detector_in_occupancy_grid_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_detector_in_occupancy_grid_lib ${catkin_LIBRARIES})

add_executable(
  collision_detector_in_occupancy_grid
  ${COLLISION_DETECTOR_IN_OCCUPANCY_GRID_SOURCE_DIR}/collision_detector_in_occupancy_grid_main.cpp
)

add_dependencies(collision_detector_in_occupancy_grid ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_detector_in_occupancy_grid collision_detector_in_occupancy_grid_lib)
target_link_libraries(collision_detector_in_occupancy_grid ${catkin_LIBRARIES})
