/*!*******************************************************************************************
 *  \file       behavior_clear_occupancy_grid.cpp
 *  \brief      Behavior Clear Occupancy Grid implementation file.
 *  \details    This file implements the BehaviorClearOccupancyGrid class.
 *  \authors    Raul Cruz, Pablo Santamaria; Melero Deza, Javier
 *  \maintainer Pablo Santamaria
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
#include "../include/behavior_clear_occupancy_grid.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorClearOccupancyGrid behavior;
  behavior.start();
  return 0;
}

BehaviorClearOccupancyGrid::BehaviorClearOccupancyGrid() : BehaviorExecutionManager()
{
  hector_reset_msg.data = "reset";
  setName("clear_occupancy_grid");
}

BehaviorClearOccupancyGrid::~BehaviorClearOccupancyGrid()
{
}

bool BehaviorClearOccupancyGrid::checkSituation() 
{ 
 return true; 
}

void BehaviorClearOccupancyGrid::checkProgress() 
{ 
 
}

void BehaviorClearOccupancyGrid::checkProcesses() 
{ 
 
}

void BehaviorClearOccupancyGrid::checkGoal() 
{
  BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
}


void BehaviorClearOccupancyGrid::onConfigure(){

  nh = getNodeHandle();
  nspace = getNamespace();
  nh.param<std::string>("poseupdate", poseupdate_topic_str, "poseupdate");
  nh.param<std::string>("self_localization/pose", pose_topic_str, "self_localization/pose");
  nh.param<std::string>("initialpose", initialpose_topic_str, "initialpose");
  nh.param<std::string>("slam_out_pose", slam_pose_topic_str, "slam_out_pose");
  is_stopped = false;
}


void BehaviorClearOccupancyGrid::onActivate(){

  is_stopped = false;
  savePoseupdate = *ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/" + nspace + "/"+poseupdate_topic_str, nh, ros::Duration(1));
  savepose = *ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/" + nspace + "/"+pose_topic_str, nh, ros::Duration(1));
  savePoseupdate.pose.pose.position =savepose.pose.position;
  savePoseupdate.pose.pose.orientation = savepose.pose.orientation;
 
  initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + nspace + "/" + initialpose_topic_str, 1, true); 

  hectorReset = nh.advertise<std_msgs::String>("/" + nspace + "/" + "syscommand", 1000, true);
  initial_pose_pub.publish(savePoseupdate);
  hectorReset.publish(hector_reset_msg);
  *ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/" + nspace + "/"+slam_pose_topic_str, nh);

  std::cout<<hector_reset_msg.data<<std::endl; 
  
}

void BehaviorClearOccupancyGrid::onDeactivate(){
  is_stopped = true;
  initial_pose_pub.shutdown();
  hectorReset.shutdown();
}

void BehaviorClearOccupancyGrid::onExecute()
{
}
