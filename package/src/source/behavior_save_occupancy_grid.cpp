/*!*******************************************************************************************
 *  \file       behavior_save_occupancy_grid.cpp
 *  \brief      Behavior Save Occupancy Grid implementation file.
 *  \details    This file implements the BehaviorClearOccupancyGrid class.
 *  \authors    Javier Melero
 *  \maintainer Javier Melero
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
#include "../include/behavior_save_occupancy_grid.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorSaveOccupancyGrid behavior;
  behavior.start();
  return 0;
}

BehaviorSaveOccupancyGrid::BehaviorSaveOccupancyGrid() : BehaviorExecutionManager()
{
  setName("save_occupancy_grid");
}

BehaviorSaveOccupancyGrid::~BehaviorSaveOccupancyGrid()
{
}

bool BehaviorSaveOccupancyGrid::checkSituation() 
{ 
 return true; 
}

void BehaviorSaveOccupancyGrid::checkProgress() 
{ 
 
}

void BehaviorSaveOccupancyGrid::checkProcesses() 
{ 
 
}

void BehaviorSaveOccupancyGrid::checkGoal() 
{
  BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
}

void BehaviorSaveOccupancyGrid::onConfigure(){
  
  is_stopped = false;
}


void BehaviorSaveOccupancyGrid::onActivate(){
  std::string nspace = getNamespace();
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  std::string save = config_file["map_name"].as<std::string>();
  std::string directory;
  const size_t last_slash_idx = save.rfind('/');
  if (std::string::npos != last_slash_idx)
  {
    directory = save.substr(0, last_slash_idx);
  }
  std::string command = "mkdir " + directory;
  dir_command = command.c_str();
  system(dir_command);
  std::cout<< directory<< std::endl;
  command = "gnome-terminal -x sh -c 'rosrun map_server map_saver --occ 90 --free 10 -f " + save + " map:=/" + nspace +"/map" + "; exec bash'";
  save_command = command.c_str();
  std::cout<< save_command<< std::endl;
  system(save_command);
}

void BehaviorSaveOccupancyGrid::onDeactivate(){
  is_stopped = true;
}

void BehaviorSaveOccupancyGrid::onExecute()
{
}
