/*
Created on Thurs Nov 16 09:36:10 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include "elfin_basic_api/elfin_basic_api_node.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_basic_api", ros::init_options::AnonymousName); // initialize the node with a random name, random name is useful when multiple nodes of the same type are running

    ros::CallbackQueue move_group_cb_queue; // ros callback queue, used to process the callback functions in a separate thread

    std::string move_group_name="elfin_arm"; // name of the move group, which is the name of the planning group in the moveit configuration file
    std::string move_group_desc="robot_description"; // name of the robot description, which is the name of the robot description in the moveit configuration file
    ros::NodeHandle move_group_nh; // ros node handle for the move group, ros node handle is used to create ros subscribers, publishers, services, etc.
    move_group_nh.setCallbackQueue(&move_group_cb_queue); // set the callback queue for the move group node handle, so that the callback functions of the move group node handle will be processed in a separate thread

    moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, move_group_nh); // create a move group options object, which is used to initialize the move group

    moveit::planning_interface::MoveGroupInterface move_group(move_group_options); // create a move group object, which is used to control the robot

    ros::AsyncSpinner move_group_spinner(1, &move_group_cb_queue); // create a ros async spinner, which is used to process the callback functions of the move group node handle in a separate thread
    move_group_spinner.start(); // start the ros async spinner
    // an async spinner is used to process the callback functions of the move group node handle in a separate thread, so that the main thread can be used to process the callback functions of other node handles

    move_group.startStateMonitor(); // start the state monitor of the move group, the state monitor is used to get the current state of the robot

    move_group.getCurrentJointValues(); // get the current joint values of the robot

    ros::AsyncSpinner common_spinner(1); // create a ros async spinner, which is used to process the callback functions of other node handles in a separate thread
    common_spinner.start(); // start the ros async spinner

    //boost::shared_ptr<tf::Transformer> tf_ptr(new tf::Transformer());

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description")); // create a robot model loader, which is used to load the robot model

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader)); // create a planning scene monitor, which is used to get the current state of the robot

    planning_scene_monitor->startSceneMonitor(); // start the scene monitor of the planning scene monitor, the scene monitor is used to get the current state of the robot
    planning_scene_monitor->startStateMonitor(); // start the state monitor of the planning scene monitor, the state monitor is used to get the current state of the robot
    planning_scene_monitor->startWorldGeometryMonitor(); // start the world geometry monitor of the planning scene monitor, the world geometry monitor is used to get the current state of the robot

    elfin_basic_api::ElfinBasicAPI basic_api(&move_group, "elfin_arm_controller/follow_joint_trajectory", planning_scene_monitor); // create an elfin basic api object, which is used to control the robot

    ros::waitForShutdown(); // wait for the shutdown signal from ROS, which is sent when Ctrl+C is pressed

    return 0;
}

