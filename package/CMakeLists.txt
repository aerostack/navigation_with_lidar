cmake_minimum_required(VERSION 2.8.3)
project(navigation_with_lidar)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(NAVIGATION_WITH_LIDAR_SOURCE_DIR
  src/source
)

set(NAVIGATION_WITH_LIDAR_INCLUDE_DIR
  src/include
)

set(NAVIGATION_WITH_LIDAR_HEADER_FILES
        ${NAVIGATION_WITH_LIDAR_INCLUDE_DIR}/behavior_clear_occupancy_grid.h
        ${NAVIGATION_WITH_LIDAR_INCLUDE_DIR}/behavior_generate_path_with_occupancy_grid.h
        ${NAVIGATION_WITH_LIDAR_INCLUDE_DIR}/behavior_save_occupancy_grid.h
)

set(NAVIGATION_WITH_LIDAR_SOURCE_FILES
        ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_clear_occupancy_grid.cpp
        ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_generate_path_with_occupancy_grid.cpp
        ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_save_occupancy_grid.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager
  droneMsgsROS
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  belief_manager_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${NAVIGATION_WITH_LIDAR_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  behavior_execution_manager
  droneMsgsROS
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  belief_manager_msgs
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(BehaviorClearOccupancyGrid ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_clear_occupancy_grid.cpp)
target_link_libraries(BehaviorClearOccupancyGrid ${catkin_LIBRARIES})
target_link_libraries(BehaviorClearOccupancyGrid yaml-cpp)
add_executable(BehaviorGeneratePathWithOccupancyGrid ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_generate_path_with_occupancy_grid.cpp)
target_link_libraries(BehaviorGeneratePathWithOccupancyGrid ${catkin_LIBRARIES})
target_link_libraries(BehaviorGeneratePathWithOccupancyGrid yaml-cpp)
add_executable(BehaviorSaveOccupancyGrid ${NAVIGATION_WITH_LIDAR_SOURCE_DIR}/behavior_save_occupancy_grid.cpp)
target_link_libraries(BehaviorSaveOccupancyGrid ${catkin_LIBRARIES})
target_link_libraries(BehaviorSaveOccupancyGrid yaml-cpp)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(BehaviorClearOccupancyGrid ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorGeneratePathWithOccupancyGrid ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorSaveOccupancyGrid ${catkin_EXPORTED_LIBRARIES})
endif()
