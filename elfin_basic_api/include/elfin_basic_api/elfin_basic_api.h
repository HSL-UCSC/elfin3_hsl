/*
Created on Mon Dec 15 10:38:07 2017

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

#ifndef ELFIN_BASIC_API_H
#define ELFIN_BASIC_API_H

#include <elfin_basic_api/elfin_teleop_api.h>
#include <elfin_basic_api/elfin_motion_api.h>
#include <elfin_basic_api/ElfinBasicAPIDynamicReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>
#include <elfin_robot_msgs/SetString.h>
#include <std_msgs/String.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <string.h>

namespace elfin_basic_api {

class ElfinBasicAPI
{
public:
    ElfinBasicAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
    // moveit is a pointer to a MoveGroupInterface object, which is used to control the robot
    // planning_interface is a pointer to a PlanningSceneInterface object, which is used to add or remove objects in the planning scene
    // action_name is the name of the action server, which is used to control the robot
    // planning_scene_monitor is a pointer to a PlanningSceneMonitor object, which is used to get the current state of the robot
    ~ElfinBasicAPI();
    // destructor

    void dynamicReconfigureCallback(ElfinBasicAPIDynamicReconfigureConfig &config, uint32_t level);
    // callback function for dynamic reconfigure, which is used to change the velocity scaling of the robot
    // config is the configuration of the dynamic reconfigure
    // level is the level of the dynamic reconfigure
    // callback means that this function will be called when the velocity scaling of the robot is changed

    void setVelocityScaling(double data);
    // set the velocity scaling of the robot, the value of which is in the range of [0, 1], and the default value is 0.1, which means the robot moves at 10% of its maximum velocity

    bool setVelocityScaling_cb(elfin_robot_msgs::SetFloat64::Request &req, elfin_robot_msgs::SetFloat64::Response &resp);
    // callback function for the service server, which is used to change the velocity scaling of the robot
    // req is the request of the service server
    // resp is the response of the service server

    bool updateVelocityScaling_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    // callback function for the service server, which is used to update the velocity scaling of the robot
    // req is the request of the service server
    // resp is the response of the service server

    bool setRefLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp);
    // callback function for the service server, which is used to set the reference link of the robot, reference link is the link that the robot moves relative to
    // example: if the reference link is "base_link" and the end link is "link_6", the robot will move relative to the base_link and move the link_6 to the target position
    
    bool setEndLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp);
    // callback function for the service server, which is used to set the end link of the robot, end link is the link that the robot moves to
    // example: if the end link is "base_link", the robot will move to the position of the base_link

    bool enableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    // callback function for the service server, which is used to enable the robot
    // std_srvs is a package that contains the service server and the service client

    bool disableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    // callback function for the service server, which is used to disable the robot

    bool stopActCtrlrs(std_srvs::SetBool::Response &resp);
    // stop the action controllers of the robot
  
    bool startElfinCtrlr(std_srvs::SetBool::Response &resp);
    // start the elfin controller of the robot

    void posePubTimer_cb(const ros::TimerEvent& evt);
    // callback function for the timer, which is used to publish the pose of the robot
    // publish for ROS means that the data is sent to the topic, and the data can be received by the subscriber of the topic
    // the topic is a channel that can be used to send and receive data
    // example: the desired position of the robot is sent to the topic, and the desired position of the robot is received from the topic

private:
    moveit::planning_interface::MoveGroupInterface *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle root_nh_, local_nh_;

    ElfinTeleopAPI *teleop_api_;
    ElfinMotionAPI *motion_api_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

    dynamic_reconfigure::Server<ElfinBasicAPIDynamicReconfigureConfig> dynamic_reconfigure_server_;

    double velocity_scaling_;

    ros::ServiceServer set_ref_link_server_;
    ros::ServiceServer set_end_link_server_;
    ros::ServiceServer enable_robot_server_;
    ros::ServiceServer disable_robot_server_;

    std::string elfin_controller_name_;
    std::vector<std::string> controller_joint_names_;

    ros::ServiceClient switch_controller_client_;
    ros::ServiceClient list_controllers_client_;
    ros::ServiceClient get_motion_state_client_;
    ros::ServiceClient get_pos_align_state_client_;

    std_srvs::SetBool::Request raw_enable_robot_request_;
    std_srvs::SetBool::Response raw_enable_robot_response_;
    ros::ServiceClient raw_enable_robot_client_;

    std_srvs::SetBool::Request raw_disable_robot_request_;
    std_srvs::SetBool::Response raw_disable_robot_response_;
    ros::ServiceClient raw_disable_robot_client_;

    std_msgs::String ref_link_name_msg_;
    std_msgs::String end_link_name_msg_;

    ros::Publisher ref_link_name_publisher_;
    ros::Publisher end_link_name_publisher_;

    tf::TransformListener tf_listener_; 
    ros::Timer pub_pose_timer; 
    ros::Publisher elfin_pose; // used to publish the pose of the robot
    tf::StampedTransform poseTransForm; // used to get the pose of the robot
    tf::Quaternion RQ; // used to get the rotation of the robot
};

}

#endif
