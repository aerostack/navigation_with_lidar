/*!*******************************************************************************************
 *  \file       collision_detector_in_occupancy_grid.cpp
 *  \brief      collision detector in occupancy grid implementation file.
 *  \details    This file contains the CollisionDetectorInOccupancyGrid implementattion of the class.
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
#include "../include/collision_detector_in_occupancy_grid.h"
#include <pluginlib/class_list_macros.h>

CollisionDetectorInOccupancyGrid::CollisionDetectorInOccupancyGrid(){
  DEBUG = false;
}

CollisionDetectorInOccupancyGrid::~CollisionDetectorInOccupancyGrid(){
}

void CollisionDetectorInOccupancyGrid::ownSetUp(){
  ros::NodeHandle private_nh("~");
  private_nh.getParam ("clearance", clearance);
  private_nh.param<std::string>("move_base_path_topic", move_base_path_topic_str, "move_base/NavfnROS/plan");
  private_nh.param<std::string>("slam_out_pose", poseupdate_topic_str, "slam_out_pose");
  //private_nh.param<std::string>("map_topic", map_topic_str, "move_base/local_costmap/costmap");
  private_nh.param<std::string>("map_topic", map_topic_str, "map");
  private_nh.param<std::string>("path_blocked_topic", path_blocked_topic_str, "environnment/path_blocked_by_obstacle");
  path_sub = node_handle.subscribe(move_base_path_topic_str, 1, &CollisionDetectorInOccupancyGrid::pathGeneratedCallback, this);
  map_sub = node_handle.subscribe(map_topic_str, 1, &CollisionDetectorInOccupancyGrid::mapCallback, this);
  poseupdate_sub = node_handle.subscribe(poseupdate_topic_str, 1, &CollisionDetectorInOccupancyGrid::poseupdateCallback, this);
  path_blocked_pub = node_handle.advertise<std_msgs::Bool>(path_blocked_topic_str, 1,true);
  std::cout << "DEBUG " << (DEBUG ? "Enabled" : "Disabled") << std::endl;
  //ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<hector_slam::dynamic_map>(nav_msgs/GetMap);
}

void CollisionDetectorInOccupancyGrid::ownStart(){
}

void CollisionDetectorInOccupancyGrid::ownRun(){}

void CollisionDetectorInOccupancyGrid::ownStop(){
  if( DEBUG ){
    std::cout << "DEBUG_MSG: Stopping"<<std::endl;
  }
  path_sub.shutdown();
  node_handle.shutdown();
}

void CollisionDetectorInOccupancyGrid::poseupdateCallback(const geometry_msgs::PoseStamped &grid_pose){
  current_grid_pose = grid_pose;
}

void CollisionDetectorInOccupancyGrid::pathGeneratedCallback(const nav_msgs::Path &resp_path){
  current_path = resp_path;
  new_path_bool = true;
}


bool CollisionDetectorInOccupancyGrid::checkPointCollision(const geometry_msgs::PoseStamped point, const double range){
  double off = 0.5;
  int x1 = ((point.pose.position.x - range - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution - off);
  int x2 = ((point.pose.position.x + range - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution - off);
  int y1 = ((point.pose.position.y - range - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution - off);
  int y2 = ((point.pose.position.y + range - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution - off);
  int width = x2-x1;
  //std::cout<< "POINTS: " << x1 << "," << x2 << "," << y1 << "," << y2 <<"\n";
  int oi , oj;
  bool obstaculo = false;
  for (int i = 0; i < width; i++){
    oi = ((y1+i)*occupancy_grid.info.width) + x1;
    for (int j = 0; j < width; j++){
      if (oi>=0 && oi<occupancy_grid.info.width*occupancy_grid.info.height){
        if (occupancy_grid.data[oi]>0){
          obstaculo = true;
          return obstaculo;
        }
        oi++;
      }
    }
  }
  return obstaculo;
}

int CollisionDetectorInOccupancyGrid::nearestN(const nav_msgs::Path path, const geometry_msgs::PoseStamped& point){
  int resultIndex = 0;
  if (path.poses.size()>0){
    double minDistance=pow(path.poses[0].pose.position.x-point.pose.position.x , 2)+pow(path.poses[0].pose.position.y-point.pose.position.y , 2); //Take the first point
    for (int i = 0; i < path.poses.size(); i++){
      double distance = pow(path.poses[i].pose.position.x-point.pose.position.x , 2)+pow(path.poses[i].pose.position.y-point.pose.position.y , 2);
      if(distance < minDistance){
        minDistance = distance;
        resultIndex = i;
      }
    }
  }
  return resultIndex;
}

double CollisionDetectorInOccupancyGrid::minDistanceToObstacle(const geometry_msgs::PoseStamped& point, const double range){
  double off = 0.5;
  int x1 = ((point.pose.position.x - range - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution - off);
  int x2 = ((point.pose.position.x + range - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution - off);
  int y1 = ((point.pose.position.y - range - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution - off);
  int y2 = ((point.pose.position.y + range - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution - off);
  int width = x2-x1;
  std::list<double> distances;
  int oi , oj;
  bool obstaculo = false;
  double minDistance = 2*pow(width , 2);
  for (int i = 0; i < width; i++){
    oi = ((y1+i)*occupancy_grid.info.width) + x1;
    for (int j = 0; j < width; j++){
      if (oi>=0 && oi<occupancy_grid.info.width*occupancy_grid.info.height){
        if (occupancy_grid.data[oi]>0){
          double distance = pow(i-width*0.5 , 2)+pow(j-width*0.5 , 2);
          if(distance < minDistance){
            obstaculo = true;
            minDistance = distance;
          }
        }
        oi++;
      }
    }
  }
  if(obstaculo == false){
    minDistance=-1;
  }
  return minDistance;
}

bool CollisionDetectorInOccupancyGrid::checkExitCollisionPath(){
  if(minDistanceToObstacle(current_path.poses[0],clearance)!=-1){
    return false;
  }
  double distances[current_path.poses.size()];
  for (int i = 0; i < current_path.poses.size(); i++){
    double dist = minDistanceToObstacle(current_path.poses[i],clearance);
    distances[i]=dist;
  }
  int i;
  for(i = 1; i< sizeof(distances) && distances[i]!=-1;i++){
    if(distances[i]<distances[i-1]){
      return false;
    }
  }
  for(i = 1; i< sizeof(distances);i++){
    if (distances[i]!=-1){
      return false;
    }
  }
  return true;
}

void CollisionDetectorInOccupancyGrid::mapCallback(const nav_msgs::OccupancyGrid &resp_occupancy_grid){
  //std::cout<< "Map is back size " << resp_occupancy_grid.data.size() <<"\n";
  occupancy_grid = resp_occupancy_grid;
  bool obstacle = false;
  int index = nearestN(current_path,current_grid_pose);
  //geometry_msgs::Pose p = current_path.poses[0].pose;
  if(current_path.poses.size() >0){
    for (int i = index; i < current_path.poses.size() && !obstacle; i++){
      if (checkPointCollision(current_path.poses[i],clearance)){
        obstacle = !checkExitCollisionPath();
      }
    }
  }
  //std::cout<< "OBSTACLE FOUND IN PATH: " << obstacle <<"\n";
  std_msgs::Bool msg;
  msg.data = obstacle;
  path_blocked_pub.publish(msg);
}
