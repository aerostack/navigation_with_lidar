/*!*******************************************************************************************
 *  \file       behavior_generate_path_with_occupancy_grid.cpp
 *  \brief      Behavior Generate Path With Occupancy Grid implementation file.
 *  \details    This file implements the BehaviorGeneratePathWithOccupancyGrid class.
 *  \authors    Raul Cruz
 *  \maintainer Raul Cruz
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
#include "../include/behavior_generate_path_with_occupancy_grid.h"
#include <thread> 

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorGeneratePathWithOccupancyGrid behavior;
  behavior.start();
  return 0;
}

BehaviorGeneratePathWithOccupancyGrid::BehaviorGeneratePathWithOccupancyGrid() : BehaviorExecutionManager()
{
  setName("generate_path_with_occupancy_grid");
  DEBUG = true;
}
BehaviorGeneratePathWithOccupancyGrid::~BehaviorGeneratePathWithOccupancyGrid()
{

}

void BehaviorGeneratePathWithOccupancyGrid::checkProgress() 
{ 

}

void BehaviorGeneratePathWithOccupancyGrid::checkProcesses() 
{ 

}

void BehaviorGeneratePathWithOccupancyGrid::onExecute() 
{
}

void BehaviorGeneratePathWithOccupancyGrid::onConfigure(){
  node_handle = getNodeHandle();
  nspace = getNamespace();
  node_handle.param<std::string>("move_base_goal_topic", move_base_goal_topic_str, "move_base_simple/goal");
  node_handle.param<std::string>("move_base_path_topic", move_base_path_topic_str, "move_base/NavfnROS/plan");
  node_handle.param<std::string>("add_belief_service_str", add_belief_service_str, "add_belief");
}

void BehaviorGeneratePathWithOccupancyGrid::onActivate(){
   // Start processes
  is_finished = false;
  bad_args = false;
  // Extract target position
  if( !grabInputPose(target_position) ){
    bad_args = true;
    is_finished = true;
    return;
  }
  belief_manager_client = node_handle.serviceClient<belief_manager_msgs::AddBelief>("/" + nspace + "/" + add_belief_service_str);
  path_sub = node_handle.subscribe("/" + nspace + "/" +move_base_path_topic_str, 1, &BehaviorGeneratePathWithOccupancyGrid::pathGeneratedCallback, this);
  goal_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/" + move_base_goal_topic_str, 1,true);
  setTimeout(true);
  timeout_thread = new boost::thread(boost::bind(&BehaviorGeneratePathWithOccupancyGrid::timeoutMonitorThread, this));
  requestPath(target_position);
}

void BehaviorGeneratePathWithOccupancyGrid::timeoutMonitorThread(){
  while(!is_finished){
    // Check whether timeout has to be generated
    boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time() - getTimeout();
    if(diff.total_milliseconds() > (2000)){
      goal_pub.publish(currentGoal);
      // Restart timeout
      setTimeout(true);
    }
    // Set timeout monitor frequency
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

boost::posix_time::ptime BehaviorGeneratePathWithOccupancyGrid::getTimeout(){
  boost::posix_time::ptime time;
  timeout_mutex.lock();
  time = initial_time;
  timeout_mutex.unlock();
  return time;
}

void BehaviorGeneratePathWithOccupancyGrid::setTimeout(bool enabled){
  timeout_mutex.lock();
  initial_time = boost::posix_time::second_clock::local_time();
  timeout_mutex.unlock();
}

bool BehaviorGeneratePathWithOccupancyGrid::generatePathCallback(aerostack_msgs::GeneratePath::Request &req, aerostack_msgs::GeneratePath::Response &res){
   goal_pub.publish(req.goal);
}

void BehaviorGeneratePathWithOccupancyGrid::checkGoal(){
  if( (is_finished && bad_args) ){
    if(DEBUG){
      std::cout<< "Behavior finished WITH error" << std::endl;
    }
    timeout_thread->interrupt();
    timeout_thread->detach();
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::TIME_OUT);
  }else if( is_finished ){
    if(DEBUG){
      std::cout<< "Behavior finished WITHOUT error" << std::endl;
    }
    timeout_thread->interrupt();
    timeout_thread->detach();
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
}

void BehaviorGeneratePathWithOccupancyGrid::onDeactivate(){
  timeout_thread->interrupt();
  timeout_thread->detach();
  goal_pub.shutdown();
  path_sub.shutdown();
  belief_manager_client.shutdown();
}

bool BehaviorGeneratePathWithOccupancyGrid::checkSituation(){
  return true;
}

// Callbacks
void BehaviorGeneratePathWithOccupancyGrid::pathGeneratedCallback(const nav_msgs::Path &resp_path){
  if(DEBUG){
    std::cout<< "Path is back, " << is_finished << std::endl;
  }
  if(!is_finished){
    // No need to convert, just reusability
    droneMsgsROS::dronePositionTrajectoryRefCommand path;
    nav_msgs::Path nav_path;
    nav_path.poses = resp_path.poses;
    convertPath(nav_path, path);
    std::string serialized = serializePath(requestBeliefId(), path);
    addBelief(serialized);
  }
}

///////////
// Utils //
///////////

int BehaviorGeneratePathWithOccupancyGrid::requestBeliefId(){
  ros::NodeHandle nh = getNodeHandle();
  std::string nspace = getNamespace();
  int ret = 100;
  belief_manager_msgs::GenerateID::Request req;
  belief_manager_msgs::GenerateID::Response res;
  ros::ServiceClient id_gen_client = nh.serviceClient<belief_manager_msgs::GenerateID>("/" + nspace + "/" + "belief_manager_process/generate_id");
  id_gen_client.call(req, res);

  if( res.ack ){
    ret = res.id;
  }

  id_gen_client.shutdown();
  return ret;
}

// Convert a navigation path to an aerostack one
void BehaviorGeneratePathWithOccupancyGrid::convertPath(
  const nav_msgs::Path &path,
  droneMsgsROS::dronePositionTrajectoryRefCommand &return_path
){
  return_path.header.stamp = ros::Time::now();
  return_path.initial_checkpoint = 0;
  return_path.is_periodic = false;

  for(int i = 0; i < path.poses.size(); i++){
    droneMsgsROS::dronePositionRefCommand next_waypoint;
    next_waypoint.x = path.poses[i].pose.position.x;
    next_waypoint.y = path.poses[i].pose.position.y;
    next_waypoint.z = target_position.z;
    if (!return_path.droneTrajectory.empty() && ((round(next_waypoint.x * 10.0 ) / 10.0) == (round (return_path.droneTrajectory.back().x *10.0)/10.0) || (round(next_waypoint.y * 10.0 ) / 10.0) == (round (return_path.droneTrajectory.back().y *10.0)/10.0))){
      continue;
    }
    else {
      return_path.droneTrajectory.push_back(next_waypoint);
    }
    
  }
}

// Convert aerostack path to string (for belief)
std::string BehaviorGeneratePathWithOccupancyGrid::serializePath(int id, droneMsgsROS::dronePositionTrajectoryRefCommand &path){
  std::ostringstream ret_stream;
  ret_stream << "path(" << id << ",(";
  for(int i = 0; i < path.droneTrajectory.size(); i++){
    ret_stream << "(";
    ret_stream << path.droneTrajectory[i].x << ",";
    ret_stream << path.droneTrajectory[i].y << ",";
    ret_stream << path.droneTrajectory[i].z;
    ret_stream << ")" << (i < path.droneTrajectory.size() -1 ? ", " : "");
  }
  ret_stream << "))";
  std::ostringstream ret_stream2;
  ret_stream2 << "object(" << id << ",path)";
  addBelief(ret_stream2.str());
  return ret_stream.str();
}

// Add a string belief
void BehaviorGeneratePathWithOccupancyGrid::addBelief(std::string belief){
  belief_manager_msgs::AddBelief belief_msg;
  belief_msg.request.multivalued = true;
  belief_msg.request.belief_expression = belief;
  bool res = false;
  while (res == false){
    res = belief_manager_client.call(belief_msg);
    if(DEBUG){
      std::cout<< "Calling service " << is_finished << std::endl;
    }  
  }
  is_finished = true;
  if(DEBUG){
    std::cout<< "Belief added " << is_finished << std::endl;
  }  
}

// Extract input arguments
bool BehaviorGeneratePathWithOccupancyGrid::grabInputPose(droneMsgsROS::dronePose &position){
  // Extract target position
  std::string arguments = getParameters();
  std::cout<<arguments<<std::endl;
  YAML::Node config_file = YAML::Load(arguments);
  if(!config_file["destination"].IsDefined()){
    std::cout<< "Null point" <<std::endl;
    return false;
  }
  std::vector<double> points = config_file["destination"].as<std::vector<double>>();
  position.x = points[0];
  position.y = points[1];
  position.z = points[2];
  if(DEBUG){
    std::cout<< "Got point ["<<position.x << ", "<< position.y << ", " << position.z <<"]"<<std::endl;
  }
  is_finished = false;
  return true;
}

// Send point to request path
bool BehaviorGeneratePathWithOccupancyGrid::requestPath(droneMsgsROS::dronePose position){
  // ask for a path to be generated
  currentGoal.header.stamp = ros::Time::now();
  currentGoal.header.frame_id = "map";
  currentGoal.pose.position.x = position.x;
  currentGoal.pose.position.y = position.y;
  currentGoal.pose.position.z = position.z;
  currentGoal.pose.orientation.x = 0.0;
  currentGoal.pose.orientation.y = 0.0;
  currentGoal.pose.orientation.z = 0.0;
  currentGoal.pose.orientation.w = 1.0;
  goal_pub.publish(currentGoal);
  return true;
}
