/*
Created on Mon Dec 15 10:58:42 2017

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

#include <elfin_basic_api/elfin_basic_api.h>

namespace elfin_basic_api {

ElfinBasicAPI::ElfinBasicAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor): 
    group_(group), action_client_(action_name, true), planning_scene_monitor_(planning_scene_monitor), local_nh_("~")
    // local_nh_ is a node handle, which is used to get parameters from the parameter server
{
    teleop_api_=new ElfinTeleopAPI(group, action_name, planning_scene_monitor);
    // teleop_api_ is a pointer to a ElfinTeleopAPI object, which is used to control the robot in teleoperation mode
    motion_api_=new ElfinMotionAPI(group, action_name, planning_scene_monitor);
    // motion_api_ is a pointer to a ElfinMotionAPI object, which is used to control the robot in motion planning mode

    dynamic_reconfigure_server_.setCallback(boost::bind(&ElfinBasicAPI::dynamicReconfigureCallback, this, _1, _2));
    // dynamic_reconfigure_server_ is a dynamic reconfigure server, which is used to change the velocity scaling factor

    set_ref_link_server_=local_nh_.advertiseService("set_reference_link", &ElfinBasicAPI::setRefLink_cb, this);
    // set_ref_link_server_ is a service server, which is used to set the reference link
    // reference link is the link that the robot moves relative to
    // "this" is a pointer to the current object, which is used to call the member function setRefLink_cb

    set_end_link_server_=local_nh_.advertiseService("set_end_link", &ElfinBasicAPI::setEndLink_cb, this);
    // set_end_link_server_ is a service server, which is used to set the end link
    // end link is the link that the robot moves to
    // "this" is a pointer to the current object, which is used to call the member function setEndLink_cb

    enable_robot_server_=local_nh_.advertiseService("enable_robot", &ElfinBasicAPI::enableRobot_cb, this);
    // enable_robot_server_ is a service server, which is used to enable the robot
    // "this" is a pointer to the current object, which is used to call the member function enableRobot_cb

    disable_robot_server_=local_nh_.advertiseService("disable_robot", &ElfinBasicAPI::disableRobot_cb, this);
    // disable_robot_server_ is a service server, which is used to disable the robot
    // "this" is a pointer to the current object, which is used to call the member function disableRobot_cb

    elfin_controller_name_=local_nh_.param<std::string>("controller_name", "elfin_arm_controller");
    // elfin_controller_name_ is the name of the default controller, which is used to control the robot in motion planning mode

    switch_controller_client_=root_nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    // switch_controller_client_ is a service client, which is used to switch controllers
    // switch between controllers is used to switch between motion planning mode and teleoperation mode
    list_controllers_client_=root_nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    // list_controllers_client_ is a service client, which is used to list controllers
    get_motion_state_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/get_motion_state");
    // get_motion_state_client_ is a service client, which is used to get the motion state of the robot
    // motion states are: moving, stopped

    get_pos_align_state_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/get_pos_align_state");
    // get_pos_align_state_client_ is a service client, which is used to get the position alignment state of the robot
    // example: if the robot is in motion planning mode, and the user sends a joint goal to the robot, the robot will move to the joint goal
    // if the robot is in teleoperation mode, and the user sends a joint goal to the robot, the robot will move to the joint goal, but the joint goal will be ignored if it is too far away from the current joint values of the robot
    // this is because the robot is in teleoperation mode, and the joint goal is not aligned with the current joint values of the robot

    raw_enable_robot_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/enable_robot");
    // raw_enable_robot_client_ is a service client, which is used to enable the robot
    // raw means that the robot is enabled without checking the motion state and position alignment state of the robot
    // this is used to enable the robot when the robot is in motion planning mode

    raw_disable_robot_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/disable_robot");
    // raw_disable_robot_client_ is a service client, which is used to disable the robot
    // raw means that the robot is disabled without checking the motion state and position alignment state of the robot
    // this is used to disable the robot when the robot is in motion planning mode

    ref_link_name_publisher_=local_nh_.advertise<std_msgs::String>("reference_link_name", 1, true);
    // ref_link_name_publisher_ is a publisher, which is used to publish the reference link name
    // publisher is used to send data to the topic
    // topic is a channel that can be used to send and receive data
    // "reference_link_name" is the name of the topic
    // 1 is the size of the message queue, which is the maximum number of messages that can be stored in the message queue
    // true means that the message queue is cleared when the publisher is shut down

    end_link_name_publisher_=local_nh_.advertise<std_msgs::String>("end_link_name", 1, true);
    // end_link_name_publisher_ is a publisher, which is used to publish the end link name
    // publisher is used to send data to the topic
    // topic is a channel that can be used to send and receive data
    // "end_link_name" is the name of the topic
    // 1 is the size of the message queue, which is the maximum number of messages that can be stored in the message queue
    // true means that the message queue is cleared when the publisher is shut down

    elfin_pose = local_nh_.advertise<geometry_msgs::Pose>("elfin_pose",1,true);
    // elfin_pose is a publisher, which is used to publish the pose of the robot
    // publisher is used to send data to the topic
    // topic is a channel that can be used to send and receive data
    // "elfin_pose" is the name of the topic
    // 1 is the size of the message queue, which is the maximum number of messages that can be stored in the message queue
    // true means that the message queue is cleared when the publisher is shut down

    ref_link_name_msg_.data=group_->getPlanningFrame();
    // ref_link_name_msg_ is a message, which is used to store the reference link name
    // group_->getPlanningFrame() is a function, which is used to get the reference link name
    
    end_link_name_msg_.data=group_->getEndEffectorLink();
    // end_link_name_msg_ is a message, which is used to store the end link name
    // group_->getEndEffectorLink() is a function, which is used to get the end link name

    ref_link_name_publisher_.publish(ref_link_name_msg_);
    // publish the reference link name to the topic
    // the reference link name can be received from the topic

    end_link_name_publisher_.publish(end_link_name_msg_);
    // publish the end link name to the topic
    // the end link name can be received from the topic

    pub_pose_timer=local_nh_.createTimer(ros::Duration(0.005), &ElfinBasicAPI::posePubTimer_cb, this);
    // pub_pose_timer is a timer, which is used to publish the pose of the robot
    // 0.005 is the duration of the timer, which is 0.005 seconds
    // "this" is a pointer to the current object, which is used to call the member function posePubTimer_cb

    pub_pose_timer.start(); // start the timer
}

ElfinBasicAPI::~ElfinBasicAPI()
{
    if(teleop_api_ != NULL)
        delete teleop_api_;
    if(motion_api_ != NULL)
        delete motion_api_;
}

void ElfinBasicAPI::dynamicReconfigureCallback(ElfinBasicAPIDynamicReconfigureConfig &config, uint32_t level)
{
    setVelocityScaling(config.velocity_scaling);
    // this is the slider in the dynamic reconfigure window, which is used to change the velocity scaling factor
}

void ElfinBasicAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
    // velocity_scaling_ is the velocity scaling factor of the robot, the value of which is in the range of [0, 1], and the default value is 0.1, which means the robot moves at 10% of its maximum velocity
    teleop_api_->setVelocityScaling(velocity_scaling_);
    motion_api_->setVelocityScaling(velocity_scaling_);
}

void ElfinBasicAPI::posePubTimer_cb(const ros::TimerEvent& evt)
{
    if(tf_listener_.waitForTransform(group_->getPlanningFrame(), group_->getEndEffectorLink(),ros::Time(0), ros::Duration(2))){
        // waitForTransform is a function, which is used to wait for the transform between the reference link and the end link
        // group_->getPlanningFrame() is a function, which is used to get the reference link name
        // group_->getEndEffectorLink() is a function, which is used to get the end link name
        // ros::Time(0) is the time of the transform
        // ros::Duration(2) is the duration of the transform, which is 2 seconds

        tf_listener_.lookupTransform(group_->getPlanningFrame(), group_->getEndEffectorLink(),ros::Time(0), poseTransForm);
        geometry_msgs::Pose msg; // msg is a message, which is used to store the pose of the robot
        // store the pose in the message as a quaternion
        // quaternion is a way to represent the orientation of the robot in 3D space

        msg.position.x = poseTransForm.getOrigin().x();
        // getOrigin is a function, which is used to get the position of the robot in the x direction
        msg.position.y = poseTransForm.getOrigin().y();
        // getOrigin is a function, which is used to get the position of the robot in the y direction
        msg.position.z = poseTransForm.getOrigin().z();
        // getOrigin is a function, which is used to get the position of the robot in the z direction
        msg.orientation.x = poseTransForm.getRotation().x();
        // getRotation is a function, which is used to get the orientation of the robot in the x direction
        msg.orientation.y = poseTransForm.getRotation().y();
        // getRotation is a function, which is used to get the orientation of the robot in the y direction
        msg.orientation.z = poseTransForm.getRotation().z();
        // getRotation is a function, which is used to get the orientation of the robot in the z direction
        msg.orientation.w = poseTransForm.getRotation().w();
        // getRotation is a function, which is used to get the orientation of the robot in the w direction
        elfin_pose.publish(msg);
        // publish the pose of the robot to the topic
    }
}

bool ElfinBasicAPI::setRefLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp)
{
    if(!tf_listener_.frameExists(req.data)) // sees if frame exists using the tf listener and passing in the frame name
    // example: if the frame name is "base_link", and there is a frame named "base_link", then frameExists will return true
    {
        resp.success=false; // if the frame does not exist, then set success to false
        std::string result="There is no frame named "; // create a string named result
        result.append(req.data); // append the frame name to the string
        resp.message=result; // set the message to the string
        return true;
    }

    teleop_api_->setRefFrames(req.data); // set the reference frame of the robot
    motion_api_->setRefFrames(req.data); // set the reference frame of the robot
    // example: if the reference frame is "base_link", and the end frame is "link_6", the robot will move relative to the base_link and move the link_6 to the target position
    ref_link_name_msg_.data=req.data; // ref_link_name_msg_ is a message, which is used to store the reference link name
    ref_link_name_publisher_.publish(ref_link_name_msg_); // publish the reference link name to the topic

    resp.success=true; // set success to true
    resp.message="Setting reference link succeed"; // set the message to "Setting reference link succeed"
    return true;
}

bool ElfinBasicAPI::setEndLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp)
{
    if(!tf_listener_.frameExists(req.data))
    {
        resp.success=false;
        std::string result="There is no frame named ";
        result.append(req.data);
        resp.message=result;
        return true;
    }

    teleop_api_->setEndFrames(req.data); // set the end frame of the robot
    motion_api_->setEndFrames(req.data); // set the end frame of the robot

    end_link_name_msg_.data=req.data; // end_link_name_msg_ is a message, which is used to store the end link name
    end_link_name_publisher_.publish(end_link_name_msg_); // publish the end link name to the topic

    resp.success=true; // set success to true
    resp.message="Setting end link succeed"; // set the message to "Setting end link succeed"
    return true;
}

bool ElfinBasicAPI::enableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    // Check request
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    // Check if there is a real driver
    if(!raw_enable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    std_srvs::SetBool::Request req_tmp; // req_tmp is a request, which is used to store the request
    std_srvs::SetBool::Response resp_tmp; // resp_tmp is a response, which is used to store the response

    // Stop active controllers
    if(!stopActCtrlrs(resp_tmp))
    {
        resp=resp_tmp; // set the response to the response of stopActCtrlrs, which is used to stop the action controllers of the robot
        // action controllers are used to control the robot in motion planning mode
        // example: if the robot is in motion planning mode, and the user sends a joint goal to the robot, the robot will move to the joint goal
        // calling stopActCtrlrs will stop the action controllers of the robot, therefore the robot will not move to the joint goal
        return true;
    }

    usleep(500000); // sleep for 0.5 seconds

    // Check motion state
    // motion state is the state of the robot, which is either moving or stopped
    if(!get_motion_state_client_.exists())
    {
        resp.message="there is no get_motion_state service";
        resp.success=false;
        return true;
    }

    req_tmp.data=true; // if motion state is true, then the robot is moving
    get_motion_state_client_.call(req_tmp, resp_tmp); // call the service client to get the motion state of the robot
    // if resp_tmp.success is true, then the robot is moving
    if(resp_tmp.success)
    {
        resp.message="failed to enable the robot, it's moving";
        resp.success=false;
        return true;
    }

    // Check position alignment state
    if(!get_pos_align_state_client_.exists())
    {
        resp.message="there is no get_pos_align_state service";
        resp.success=false;
        return true;
    }

    req_tmp.data=true; // if position alignment state is true, then the robot is in teleoperation mode
    get_pos_align_state_client_.call(req_tmp, resp_tmp);
    if(!resp_tmp.success)
    {
        resp.message="failed to enable the robot, commands aren't aligned with actual positions";
        resp.success=false;
        return true;
    }

    // Check enable service
    if(!raw_enable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    // Enable servos
    raw_enable_robot_request_.data=true; // if raw_enable_robot_request_.data is true, then the robot is enabled
    raw_enable_robot_client_.call(raw_enable_robot_request_, raw_enable_robot_response_);
    // if raw_enable_robot_request_ is true, then the robot is enabled
    // if raw_enable_robot_response_ is true, then the robot is enabled
    resp=raw_enable_robot_response_;

    // Start default controller
    if(!startElfinCtrlr(resp_tmp)) // default controller on gui is elfin_arm_controller, which is used to control the robot in motion planning mode
    {
        resp=resp_tmp;
        return true;
    }

    return true;
}

bool ElfinBasicAPI::disableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    // Check request
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    // Check disable service
    if(!raw_disable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    // Disable servos
    raw_disable_robot_request_.data=true;
    raw_disable_robot_client_.call(raw_disable_robot_request_, raw_disable_robot_response_);
    resp=raw_disable_robot_response_;

    std_srvs::SetBool::Response resp_tmp;

    // Stop active controllers
    if(!stopActCtrlrs(resp_tmp))
    {
        resp=resp_tmp;
        return true;
    }

    return true;
}

bool ElfinBasicAPI::stopActCtrlrs(std_srvs::SetBool::Response &resp)
{
    // Check list controllers service
    if(!list_controllers_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }

    // Find controllers to stop
    controller_manager_msgs::ListControllers::Request list_controllers_request;
    controller_manager_msgs::ListControllers::Response list_controllers_response;
    list_controllers_client_.call(list_controllers_request, list_controllers_response);
    std::vector<std::string> controllers_to_stop; // add controllers from list_controllers_response to controllers_to_stop
    controllers_to_stop.clear(); // clear the controllers_to_stop vector

    controller_joint_names_.clear(); // clear the controller_joint_names_ vector
    for(int i=0; i<list_controllers_response.controller.size(); i++) // loop through the controllers in list_controllers_response
    {
        std::string name_tmp=list_controllers_response.controller[i].name; // get the name of the controller
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources; // get the resources of the controller
        // resources are the joints of the robot
        if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0) // if the name of the controller is elfin_controller_name_, which is elfin_arm_controller, then add the resources of the controller to controller_joint_names_
        {
            for(int j=0; j<resrc_tmp.size(); j++) // loop through the resources of the controller
            {
                controller_joint_names_.insert(controller_joint_names_.end(), resrc_tmp[j].resources.begin(),
                                               resrc_tmp[j].resources.end());
                                               // insert the resources of the controller to the end of the controller_joint_names_ vector
            }
            break;
        }
    }

    for(int i=0; i<list_controllers_response.controller.size(); i++) // loop through the controllers in list_controllers_response
    {
        std::string state_tmp=list_controllers_response.controller[i].state; // get the state of the controller
        std::string name_tmp=list_controllers_response.controller[i].name; // get the name of the controller
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        // get the resources of the controller, resources include the joints of the robot
        if(strcmp(state_tmp.c_str(), "running")==0) // if the state of the controller is running, then add the controller to controllers_to_stop
        {
            bool break_flag=false; // break_flag is a boolean, which is used to break out of the loop
            for(int j=0; j<resrc_tmp.size(); j++) // loop through the resources of the controller
            {
                for(int k=0; k<controller_joint_names_.size(); k++) // loop through the controller_joint_names_ vector
                {
                    if(std::find(resrc_tmp[j].resources.begin(), resrc_tmp[j].resources.end(), // if the resource of the controller is in the controller_joint_names_ vector, then add the controller to controllers_to_stop
                                 controller_joint_names_[k])!=resrc_tmp[j].resources.end())
                    {
                        break_flag=true; // break out of the loop
                        controllers_to_stop.push_back(name_tmp); // add the controller to controllers_to_stop
                    }
                    if(break_flag)
                    {
                        break;
                    }
                }
                if(break_flag)
                {
                    break;
                }
            }
        }
    }

    // Stop active controllers
    if(controllers_to_stop.size()>0)
    {
        // Check switch controller service
        if(!switch_controller_client_.exists())
        {
            resp.message="there is no controller manager";
            resp.success=false;
            return false;
        }

        // Stop active controllers
        controller_manager_msgs::SwitchController::Request switch_controller_request;
        controller_manager_msgs::SwitchController::Response switch_controller_response;
        switch_controller_request.start_controllers.clear();
        switch_controller_request.stop_controllers=controllers_to_stop;
        switch_controller_request.strictness=switch_controller_request.STRICT;

        switch_controller_client_.call(switch_controller_request, switch_controller_response);
        if(!switch_controller_response.ok)
        {
            resp.message="Failed to stop active controllers";
            resp.success=false;
            return false;
        }
    }

    return true;
}

bool ElfinBasicAPI::startElfinCtrlr(std_srvs::SetBool::Response &resp)
{
    // Check switch controller service
    if(!switch_controller_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }

    // Start active controllers
    controller_manager_msgs::SwitchController::Request switch_controller_request;
    controller_manager_msgs::SwitchController::Response switch_controller_response;
    switch_controller_request.start_controllers.clear();
    switch_controller_request.start_controllers.push_back(elfin_controller_name_);
    switch_controller_request.stop_controllers.clear();
    switch_controller_request.strictness=switch_controller_request.STRICT;

    switch_controller_client_.call(switch_controller_request, switch_controller_response);
    if(!switch_controller_response.ok)
    {
        resp.message="Failed to start the default controller";
        resp.success=false;
        return false;
    }

    return true;
}

}
