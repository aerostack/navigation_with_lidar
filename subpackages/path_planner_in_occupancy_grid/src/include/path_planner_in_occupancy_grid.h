/*!*******************************************************************************************
 *  \file       path_planner_in_occupancy_grid.h
 *  \brief      path planner in occupancy grid definition file.
 *  \details    This file contains the PathPlannerInOccupancyGrid declaration. To obtain more information about
 *              it's definition consult the path_planner_in_occupancy_grid.cpp file.
 *  \authors    Raul Cruz
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
#ifndef PATH_PLANNER_IN_OCCUPANCY_GRID_H
#define PATH_PLANNER_IN_OCCUPANCY_GRID_H

#include <string>
#include <queue>

// ROS
#include <ros/ros.h>

// Move base
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>

// Aerostack
#include <robot_process.h>
#include <aerostack_msgs/GeneratePath.h>
#include <aerostack_msgs/PathWithID.h>

typedef std::pair<unsigned int, geometry_msgs::PoseStamped> GoalWithId;
typedef std::queue<GoalWithId> GoalQueue;

class PathPlannerInOccupancyGrid : public RobotProcess
{
public:
  PathPlannerInOccupancyGrid();
  ~PathPlannerInOccupancyGrid();

private:

  const double MONITOR_FREQ = 0.5; // Hz
  double TIMEOUT_VALUE; // Seconds

  bool DEBUG;

  GoalWithId current_goal;
  GoalQueue goal_queue;
  unsigned int current_id;

  //positions
  double world_x;
  double world_y;
  double world_z;
  double grid_x;
  double grid_y;
  double grid_z;

  ros::NodeHandle node_handle;
  ros::ServiceServer goal_server;
  ros::Publisher goal_pub;
  ros::Publisher path_pub;
  ros::Subscriber path_sub;
  ros::Subscriber world_pose_sub;
  ros::Subscriber slam_out_pose_sub;

  // Thread related
  boost::mutex goal_mutex, timeout_mutex;
  boost::thread *timeout_thread;
  bool running_goal;

  // Timeout monitor related
  boost::posix_time::ptime initial_time;

  // Congfig variables
  std::string drone_id;
  std::string drone_id_namespace;
  std::string my_stack_directory;
  
  // Topic & server names
  std::string move_base_goal_topic_str;
  std::string move_base_path_topic_str;
  std::string self_localization_pose_topic_str;
  std::string slam_out_pose_topic_str;
  std::string output_path_topic_str;
  std::string new_goal_server_str;

  GoalWithId getCurrentGoal();
  void setCurrentGoal(GoalWithId);
  boost::posix_time::ptime getTimeout();
  void setTimeout(bool);
  bool isTimeoutEnabled();

  // DroneProcess
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();

  // Threads
  void timeoutMonitorThread();

  // Callbacks
  void pathGeneratedCallback(const nav_msgs::Path &path);
  void self_localization_pose(const geometry_msgs::PoseStamped &world_pose);
  void slam_out_pose(const geometry_msgs::PoseStamped &grid_pose);
  bool generatePathCallback(aerostack_msgs::GeneratePath::Request &req,
                            aerostack_msgs::GeneratePath::Response &res);
};

#endif
