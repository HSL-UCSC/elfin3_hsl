/*
Created on Mon Nov 27 14:24:30 2017

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

#ifndef ELFIN_MOTION_API_H
#define ELFIN_MOTION_API_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <elfin_basic_api/elfin_basic_api_const.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace elfin_basic_api {

class ElfinMotionAPI
{
public:
    ElfinMotionAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
    void jointGoalCB(const sensor_msgs::JointStateConstPtr &msg);
    // callback function for joint goal, which is used to control the robot to a joint goal
    // provides a message of type sensor to tell elfin which joint setpoints to go to

    void cartGoalCB(const geometry_msgs::PoseStampedConstPtr &msg);
    // callback function for cartesian goal, which is used to control the robot to a cartesian goal
    // provides a message of type geometry_msgs::PoseStamped to tell elfin which cartesian setpoints to go to

    void trajectoryScaling(moveit_msgs::RobotTrajectory &trajectory, double scale);
    // scale the velocity of the trajectory
    // example: trajectoryScaling(trajectory, 0.5) will scale the velocity of the trajectory to half of the original velocity

    void cartPathGoalCB(const geometry_msgs::PoseArrayConstPtr &msg);
    // callback function for cartesian path goal, which is used to control the robot to a cartesian path goal
    // allows the user to specify a path in cartesian space for the robot to follow
    // provides a message of type geometry_msgs::PoseArray to tell elfin which cartesian setpoints to go to

    bool getRefLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool getEndLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    void torquesPubTimer_cb(const ros::TimerEvent& evt);

    void setVelocityScaling(double data);
    void setRefFrames(std::string ref_link);
    void setEndFrames(std::string end_link);

    bool updateTransforms(std::string ref_link);
    // update the transforms between the reference link and the root link, and between the end link and the tip link
    // ref_link is the reference link of the robot, which is the link that the robot moves relative to
    // tip link is the end link of the robot, which is the link that the robot moves to

private:
    moveit::planning_interface::MoveGroupInterface *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    dynamics_solver::DynamicsSolverPtr dynamics_solver_;
    ros::NodeHandle root_nh_, motion_nh_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

    double velocity_scaling_;

    ros::Subscriber joint_goal_sub_;
    ros::Subscriber cart_goal_sub_;
    ros::Subscriber cart_path_goal_sub_;

    std::string end_link_;
    std::string reference_link_;
    std::string default_tip_link_;
    std::string root_link_;

    tf::TransformListener tf_listener_;

    tf::StampedTransform transform_rootToRef_;
    tf::StampedTransform transform_tipToEnd_;

    ros::ServiceServer get_reference_link_server_;
    ros::ServiceServer get_end_link_server_;
    ros::ServiceServer get_torques_server_;

    ros::Timer torques_publisher_timer_;
    ros::Publisher torques_publisher_;
    std_msgs::Float64MultiArray torques_msg_;

};

}

#endif
