/*!*******************************************************************************************
 *  \file       collision_detector_in_occupancy_grid.h
 *  \brief      collision detector in occupancy grid definition file.
 *  \details    This file contains the CollisionDetectorInOccupancyGrid declaration. To obtain more information about
 *              it's definition consult the collision_detector_in_occupancy_grid.cpp file.
 *  \authors    Pablo Santamaria
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#ifndef COLLISION_DETECTOR_IN_OCCUPANCY_GRID_H
#define COLLISION_DETECTOR_IN_OCCUPANCY_GRID_H

#include <string>
#include <queue>

// ROS
#include <ros/ros.h>

// Move base
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>

// Aerostack
#include <robot_process.h>
#include <behavior_coordinator_msgs/StopTask.h>

#include <aerostack_msgs/GeneratePath.h>
#include <aerostack_msgs/PathWithID.h>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"

typedef std::pair<unsigned int, geometry_msgs::PoseStamped> GoalWithId;
typedef std::queue<GoalWithId> GoalQueue;

class CollisionDetectorInOccupancyGrid : public RobotProcess
{
public:
  CollisionDetectorInOccupancyGrid();
  ~CollisionDetectorInOccupancyGrid();

  // DroneProcess
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();

private:

  bool DEBUG;

  ros::NodeHandle node_handle;
  ros::Subscriber path_sub;
  ros::Subscriber map_sub;
  ros::Subscriber poseupdate_sub;
  ros::Publisher path_blocked_pub;
  ros::ServiceClient stop_task_srv;

  nav_msgs::Path current_path;
  nav_msgs::OccupancyGrid occupancy_grid;
  double distance;
  double clearance;
  bool new_path_bool;
  nav_msgs::OccupancyGridConstPtr map;
  geometry_msgs::PoseStamped current_grid_pose;

  // Topic & server names
  std::string move_base_path_topic_str;
  std::string map_topic_str;
  std::string poseupdate_topic_str;
  std::string path_blocked_topic_str;
  std::string stop_task;

  // Callbacks
  void pathGeneratedCallback(const nav_msgs::Path &path);
  void mapCallback(const nav_msgs::OccupancyGrid &resp_occupancy_grid);
  void poseupdateCallback(const geometry_msgs::PoseStamped &grid_pose);
  bool checkPointCollision(const geometry_msgs::PoseStamped point, const double range);
  int nearestN(const nav_msgs::Path path, const geometry_msgs::PoseStamped& point);
  bool checkExitCollisionPath();
  double minDistanceToObstacle(const geometry_msgs::PoseStamped& point, const double range);
};

#endif
