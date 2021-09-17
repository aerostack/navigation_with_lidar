/*!*******************************************************************************************
 *  \file       path_planner_in_occupancy_grid.cpp
 *  \brief      path planner in occupancy grid implementation file.
 *  \details    This file contains the PathPlannerInOccupancyGrid implementattion of the class.
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
#include "../include/path_planner_in_occupancy_grid.h"

PathPlannerInOccupancyGrid::PathPlannerInOccupancyGrid() : RobotProcess(){
  current_id = 0;
  running_goal = false;
  DEBUG = false;
  TIMEOUT_VALUE = 10;
}

PathPlannerInOccupancyGrid::~PathPlannerInOccupancyGrid(){
}

void PathPlannerInOccupancyGrid::ownSetUp(){
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("move_base_goal_topic", move_base_goal_topic_str, "move_base_simple/goal");
  private_nh.param<std::string>("move_base_path_topic", move_base_path_topic_str, "move_base/NavfnROS/plan");
  private_nh.param<std::string>("self_localization_pose_topic", self_localization_pose_topic_str, "self_localization/pose");
  private_nh.param<std::string>("slam_out_pose", slam_out_pose_topic_str, "slam_out_pose");
  private_nh.param<std::string>("output_path_topic", output_path_topic_str, "path_with_id");
  private_nh.param<std::string>("generate_path_service", new_goal_server_str, "generate_path");
  private_nh.param<bool>("debug", DEBUG, false);
  private_nh.param<double>("timeout", TIMEOUT_VALUE, 10);
  world_pose_sub = node_handle.subscribe(self_localization_pose_topic_str, 1, &PathPlannerInOccupancyGrid::self_localization_pose, this);
  slam_out_pose_sub = node_handle.subscribe(slam_out_pose_topic_str, 1, &PathPlannerInOccupancyGrid::slam_out_pose, this);
  std::cout << "DEBUG " << (DEBUG ? "Enabled" : "Disabled") << std::endl;
}

void PathPlannerInOccupancyGrid::ownStart(){
  // Start service
  goal_server = node_handle.advertiseService(new_goal_server_str, &PathPlannerInOccupancyGrid::generatePathCallback, this);
  
  // Subscribe & publish to topics
  goal_pub = node_handle.advertise<geometry_msgs::PoseStamped>(move_base_goal_topic_str, 1);
  path_pub = node_handle.advertise<aerostack_msgs::PathWithID>(output_path_topic_str, 1);
  path_sub = node_handle.subscribe(move_base_path_topic_str, 1, &PathPlannerInOccupancyGrid::pathGeneratedCallback, this);

  // Generate timeout thread
  timeout_thread = new boost::thread(boost::bind(&PathPlannerInOccupancyGrid::timeoutMonitorThread, this));

  if( DEBUG ){
    std::cout << "DEBUG_MSG: Tiemout thread generated" << std::endl;
  }
}

void PathPlannerInOccupancyGrid::self_localization_pose(const geometry_msgs::PoseStamped &world_pose){
  world_x = world_pose.pose.position.x;
  world_y = world_pose.pose.position.y;
}

void PathPlannerInOccupancyGrid::slam_out_pose(const geometry_msgs::PoseStamped &grid_pose){
  grid_x = grid_pose.pose.position.x;
  grid_y = grid_pose.pose.position.y;
}

void PathPlannerInOccupancyGrid::ownRun(){}

void PathPlannerInOccupancyGrid::ownStop(){
  // Stop topics, server & thread
  setTimeout(false);

  if( DEBUG ){
    std::cout << "DEBUG_MSG: Stopping, queue size ("<< goal_queue.size() <<")"<<std::endl;
  }
  goal_server.shutdown();
  timeout_thread->interrupt();
  timeout_thread->detach();
  goal_pub.shutdown();
  path_sub.shutdown();
  path_pub.shutdown();
  world_pose_sub.shutdown();
  slam_out_pose_sub.shutdown();
  node_handle.shutdown();
}

GoalWithId PathPlannerInOccupancyGrid::getCurrentGoal(){
  GoalWithId goal;
  goal_mutex.lock();
  goal = current_goal;
  goal_mutex.unlock();
  return goal;
}

void PathPlannerInOccupancyGrid::setCurrentGoal(GoalWithId goal){
  goal_mutex.lock();
  current_goal = goal;
  running_goal = true;
  goal_mutex.unlock();
}

boost::posix_time::ptime PathPlannerInOccupancyGrid::getTimeout(){
  boost::posix_time::ptime time;
  timeout_mutex.lock();
  time = initial_time;
  timeout_mutex.unlock();
  return time;
}

void PathPlannerInOccupancyGrid::setTimeout(bool enabled){
  timeout_mutex.lock();
  initial_time = boost::posix_time::second_clock::local_time();
  running_goal = enabled;
  timeout_mutex.unlock();
}

bool PathPlannerInOccupancyGrid::isTimeoutEnabled(){
  bool ret;
  timeout_mutex.lock();
  ret = running_goal;
  timeout_mutex.unlock();
  return ret;
}

void PathPlannerInOccupancyGrid::timeoutMonitorThread(){
  while(true){
    // Check whether timeout has to be generated
    boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time() - getTimeout();
    if( (diff.total_milliseconds() > (TIMEOUT_VALUE * 1000)) && isTimeoutEnabled() ){
      //path_sub = node_handle.subscribe(move_base_path_topic_str, 1, &PathPlannerInOccupancyGrid::pathGeneratedCallback, this);
      // Re-send previous goal to move_base
      GoalWithId goal = getCurrentGoal();
      unsigned int goal_id;
      geometry_msgs::PoseStamped pose_goal;
      std::tie(goal_id, pose_goal) = goal;

      pose_goal.header.stamp = ros::Time::now();
      // ToDo := Check this out
      pose_goal.header.frame_id = "map";

      pose_goal.pose.orientation.x = 0;
      pose_goal.pose.orientation.y = 0;
      pose_goal.pose.orientation.z = 0;
      pose_goal.pose.orientation.w = 1;

      // Publish the subgoal
      goal_pub.publish(pose_goal);

      // Restart timeout
      setTimeout(true);
      if( DEBUG ){
        std::cout << "DEBUG_MSG: Timeout reached, resending goal to move base node" << std::endl;
      }
    }
    // Set timeout monitor frequency
    boost::this_thread::sleep(boost::posix_time::milliseconds(int((1.d/MONITOR_FREQ) * 1000)));
  }
}

void PathPlannerInOccupancyGrid::pathGeneratedCallback(const nav_msgs::Path &path){
  //path_sub.shutdown();
  GoalWithId last_goal = getCurrentGoal();
  nav_msgs::Path newPath;
  newPath.poses = path.poses;
  for(int i = 0; i<path.poses.size(); i++){
    newPath.poses[i].pose.position.x = path.poses[i].pose.position.x + world_x - grid_x;
    newPath.poses[i].pose.position.y = path.poses[i].pose.position.y + world_y - grid_y;
  }
  aerostack_msgs::PathWithID response;
  response.uid = last_goal.first;
  response.header = path.header;
  response.poses = newPath.poses;
  path_pub.publish(response);

  if( DEBUG ){
    std::cout << "DEBUG_MSG: Path arrived, sending with id (" << last_goal.first << "), path size (" << path.poses.size() << "), queue size ("<< goal_queue.size() <<")"<<std::endl;
  }
  
  // reset timeout, enable/disable when goals remaining
  setTimeout(goal_queue.size() > 0);
  if( goal_queue.size() > 0 ){

    // dequeue, only write-access for the queue, no need to sync
    GoalWithId new_goal = goal_queue.front();
    goal_queue.pop();
    setCurrentGoal(new_goal);
    goal_pub.publish(new_goal.second);
  }
}

bool PathPlannerInOccupancyGrid::generatePathCallback(
  aerostack_msgs::GeneratePath::Request &req,
  aerostack_msgs::GeneratePath::Response &res
){
  // new goal arrived
  // grab new id and set response
  current_id++;
  res.uid = current_id;

  std::cout << "GRID: " << grid_x << "," << grid_y << std::endl;
  std::cout << "WORLD: " << world_x << "," << world_y << std::endl;
  std::cout << "OFFSETS: " << grid_x - world_x << "," << grid_y - world_y << std::endl;
  req.goal.pose.position.x = req.goal.pose.position.x + grid_x - world_x;
  req.goal.pose.position.y = req.goal.pose.position.y + grid_y - world_y;
  std::cout << req.goal.pose.position <<std::endl;
  GoalWithId new_goal = std::make_pair(current_id, req.goal);
  if( DEBUG ){
    std::cout << "DEBUG_MSG: Goal arrived, assigned id (" << current_id << ") queue size ("<< goal_queue.size() <<")"<<std::endl;
  }
  // if running goal, just enqueue, else, set it up
  if( isTimeoutEnabled() ){
    goal_queue.push(new_goal);
  }else{
    setCurrentGoal(new_goal);
    goal_pub.publish(req.goal);
  }
  return true;
}
