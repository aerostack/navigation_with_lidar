/*!*******************************************************************************************
 *  \file       behavior_clear_occupancy_grid.h
 *  \brief      Behavior Clear Occupancy Grid implementation file.
 *  \details    This file implements the BehaviorClearOccupancyGrid class.
 *  \authors    Raul Cruz, Pablo Santamaria; Melero Deza, Javier
 *  \maintainer Pablo Santamaria
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
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
* THIS SOFTWARE IS PROVIDED With THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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
#ifndef behavior_clear_occupancy_grid_H
#define behavior_clear_occupancy_grid_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <droneMsgsROS/dronePose.h>

#include <BehaviorExecutionManager.h>
#include <geometry_msgs/PoseStamped.h>

class BehaviorClearOccupancyGrid : public BehaviorExecutionManager
{
public:
  BehaviorClearOccupancyGrid();
  ~BehaviorClearOccupancyGrid();

private:

  std::string self_localization_pose_str;
  std::string nspace; 
  ros::Subscriber self_localization_sub;
  geometry_msgs::PoseWithCovarianceStamped poseupdate;
  geometry_msgs::PoseWithCovarianceStamped savePoseupdate;
  geometry_msgs::PoseStamped slam_pose;
  geometry_msgs::PoseStamped savepose;

  geometry_msgs::PoseWithCovarianceStamped initialpose;

  ros::Publisher hectorReset;
  std_msgs::String hector_reset_msg;
  
  ros::NodeHandle nh;
  ros::Publisher initial_pose_pub;
  ros::Publisher slam_pose_pub;
  ros::Publisher poseupdate_pub;
  ros::Publisher initialpose_pub;
  ros::Subscriber slam_pose_sub;
  ros::Subscriber poseupdate_sub;
  void PoseUpdateCallback (const geometry_msgs::PoseWithCovarianceStamped msg);
  void PoseCallback (const geometry_msgs::PoseStamped msg);

  //Congfig variables

  std::string hector_map_topic_str;
  std::string slam_pose_topic_str;
  std::string poseupdate_topic_str;
  std::string initialpose_topic_str;
  std::string pose_topic_str;
  bool is_stopped;


  // BehaviorExecutionManager
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
};
#endif