/*
Created on Mon Nov 27 14:22:43 2017

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

#include "elfin_basic_api/elfin_motion_api.h"

namespace elfin_basic_api {

ElfinMotionAPI::ElfinMotionAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
    group_(group), action_client_(action_name, true), planning_scene_monitor_(planning_scene_monitor), motion_nh_("~")
{
    geometry_msgs::Vector3 gravity_v3; // gravity vector for dynamics solver
    gravity_v3.x=0; // x component of gravity vector
    gravity_v3.y=0; // y component of gravity vector
    gravity_v3.z=-9.81; // z component of gravity vector

    dynamics_solver_.reset(new dynamics_solver::DynamicsSolver(group->getRobotModel(), group->getName(), gravity_v3));
    // dynamics_solver_ is a pointer to a DynamicsSolver object, which is used to compute the torques for the robot

    goal_.trajectory.joint_names=group_->getJointNames(); // get the names of the joints of the robot
    goal_.trajectory.header.stamp.sec=0; // set the seconds component of the time stamp of the goal
    goal_.trajectory.header.stamp.nsec=0; // set the nanoseconds component of the time stamp of the goal
    //.header is a member of the trajectory_msgs::JointTrajectory object, which is used to store the header of the trajectory
    //.stamp is a member of the std_msgs::Header object, which is used to store the time stamp of the header

    joint_goal_sub_=motion_nh_.subscribe("joint_goal", 1, &ElfinMotionAPI::jointGoalCB, this);
    // joint_goal_sub_ is a subscriber, which is used to receive the joint goal
    cart_goal_sub_=motion_nh_.subscribe("cart_goal", 1, &ElfinMotionAPI::cartGoalCB, this);
    // cart_goal_sub_ is a subscriber, which is used to receive the cartesian goal
    cart_path_goal_sub_=motion_nh_.subscribe("cart_path_goal", 1, &ElfinMotionAPI::cartPathGoalCB, this);
    // cart_path_goal_sub_ is a subscriber, which is used to receive the cartesian path goal
    // difference between path and non-path goals: path goals are used to specify a path for the robot to follow, while non-path goals are used to specify a single pose for the robot to reach

    get_reference_link_server_=motion_nh_.advertiseService("get_reference_link", &ElfinMotionAPI::getRefLink_cb, this);
    get_end_link_server_=motion_nh_.advertiseService("get_end_link", &ElfinMotionAPI::getEndLink_cb, this);
    //.advertiseService is used to create a service server, a service is used to provide to other nodes a function that can be called remotely

    bool torques_publisher_flag=motion_nh_.param<bool>("torques_publish", false);
    // .param is used to get a parameter from the parameter server, if the parameter does not exist, the default value will be used
    double torques_publisher_period=motion_nh_.param<double>("torques_publish_period", 0.02);
    // get the period of the torques publisher, the default value is 0.02, which means the torques publisher publishes the torques every 0.02 seconds
    // the torques are computed by the dynamics solver in the callback function of the torques publisher timer

    if(torques_publisher_flag)
    {
        torques_publisher_=motion_nh_.advertise<std_msgs::Float64MultiArray>("desired_torques", 1);
        // torques_publisher_ is a publisher, which is used to publish the torques in a message of type std_msgs::Float64MultiArray
        torques_publisher_timer_=motion_nh_.createTimer(ros::Duration(torques_publisher_period), &ElfinMotionAPI::torquesPubTimer_cb, this);
        // torques_publisher_timer_ is a timer, which is used to publish the torques periodically, the period is specified by the parameter "torques_publish_period"
    }

    end_link_=group_->getEndEffectorLink();
    // get the end link of the robot, which is the link that the robot moves relative to
    reference_link_=group_->getPlanningFrame();
    // get the reference link of the robot, which is the link that the robot moves to

    default_tip_link_=group_->getEndEffectorLink();
    // get the default tip link of the robot, which is the link that the robot moves to
    root_link_=group->getPlanningFrame();
    // get the root link of the robot, which is the link that the robot moves relative to
}

void ElfinMotionAPI::jointGoalCB(const sensor_msgs::JointStateConstPtr &msg)
{
    if(group_->setJointValueTarget(*msg))
    {
        group_->asyncMove();
        // asyncMove is a function of the MoveGroupInterface class, which is used to move the robot asynchronously
        // asynchrously being that the function returns immediately, and the robot moves in the background
    }
    else
    {
        ROS_WARN("the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::cartGoalCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
    std::string reference_link=reference_link_;
    if(!msg->header.frame_id.empty())
    // if the frame id of the message is not empty, use the frame id of the message as the reference link
    {
        reference_link=msg->header.frame_id;
    }

    if(!updateTransforms(reference_link))
        return;

    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    // Eigen::Isometry3d is a type of object that is used to store the affine transformation matrix
    // affine transformation matrix is a matrix that is used to represent the translation and rotation of an object
    tf::transformTFToEigen(transform_rootToRef_, affine_rootToRef);
    // transformTFToEigen is a function of the tf::Transformer class, which is used to convert a transform of type tf::StampedTransform to an affine transformation matrix of type Eigen::Isometry3d
    affine_refToRoot=affine_rootToRef.inverse();
    // .inverse is a function of the Eigen::Isometry3d class, which is used to get the inverse of the affine transformation matrix

    Eigen::Isometry3d affine_tipToEnd; // tiptoend is the transform between the tip link and the end link
    tf::transformTFToEigen(transform_tipToEnd_, affine_tipToEnd);

    tf::Pose tf_pose_tmp;
    // used to store the pose of the robot
    Eigen::Isometry3d affine_pose_tmp;
    // used to store the affine transformation matrix of the pose of the robot
    tf::poseMsgToTF(msg->pose, tf_pose_tmp);
    // used to convert a pose of type geometry_msgs::Pose to a pose of type tf::Pose
    tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp);
    // used to convert a pose of type tf::Pose to an affine transformation matrix of type Eigen::Isometry3d
    // converts the pose of the robot to an affine transformation matrix
    // example: the pose of the robot is [0, 0, 0, 0, 0, 0], which means the robot is at the home position, and the affine transformation matrix is an identity matrix

    Eigen::Isometry3d affine_pose_goal=affine_refToRoot * affine_pose_tmp * affine_tipToEnd;
    // get the affine transformation matrix of the goal pose of the robot

    if(group_->setPoseTarget(affine_pose_goal))
    {
        group_->asyncMove();
    }
    else
    {
        ROS_WARN("the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::trajectoryScaling(moveit_msgs::RobotTrajectory &trajectory, double scale)
{
    if(scale<=0 || scale>=1) // if the scale is not in the range of (0, 1), return
    {
        return;
    }
    for(int i=0; i<trajectory.joint_trajectory.points.size(); i++) // for each point in the trajectory
    {
        trajectory.joint_trajectory.points[i].time_from_start.operator *=(1.0/scale); // scale the time from start of the point
        for(int j=0; j<trajectory.joint_trajectory.points[i].velocities.size(); j++) // for each velocity in the point
        {
            trajectory.joint_trajectory.points[i].velocities[j]*=scale; // scale the velocity of the point
        }
        for(int j=0; j<trajectory.joint_trajectory.points[i].accelerations.size(); j++) // for each acceleration in the point
        {
            trajectory.joint_trajectory.points[i].accelerations[j]*=(scale*scale); // scale the acceleration of the point
        }
    }
}

void ElfinMotionAPI::cartPathGoalCB(const geometry_msgs::PoseArrayConstPtr &msg)
{
    moveit_msgs::RobotTrajectory cart_path;
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;

    std::vector<geometry_msgs::Pose> pose_goal=msg->poses;

    std::string reference_link=reference_link_;
    if(!msg->header.frame_id.empty())
    {
        reference_link=msg->header.frame_id;
    }

    if(!updateTransforms(reference_link))
        return;

    Eigen::Isometry3d affine_rootToRef, affine_refToRoot; // root is the root link of the robot
    tf::transformTFToEigen(transform_rootToRef_, affine_rootToRef); // get the affine transformation matrix of the transform between the reference link and the root link
    affine_refToRoot=affine_rootToRef.inverse(); // inverse the affine transformation matrix

    Eigen::Isometry3d affine_tipToEnd; // tiptoend is the transform between the tip link and the end link
    tf::transformTFToEigen(transform_tipToEnd_, affine_tipToEnd); // get the affine transformation matrix of the transform between the tip link and the end link

    tf::Pose tf_pose_tmp;
    Eigen::Isometry3d affine_pose_tmp;
    Eigen::Isometry3d affine_goal_tmp;

    for(int i=0; i<pose_goal.size(); i++) // for each pose in the path
    {
        tf::poseMsgToTF(pose_goal[i], tf_pose_tmp); // convert the pose of type geometry_msgs::Pose to a pose of type tf::Pose
        tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp); // convert the pose of type tf::Pose to an affine transformation matrix of type Eigen::Isometry3d

        affine_goal_tmp=affine_refToRoot * affine_pose_tmp * affine_tipToEnd; // get the affine transformation matrix of the goal pose of the robot

        tf::poseEigenToTF(affine_goal_tmp, tf_pose_tmp); // convert the affine transformation matrix of type Eigen::Isometry3d to a pose of type tf::Pose
        tf::poseTFToMsg(tf_pose_tmp, pose_goal[i]); // convert the pose of type tf::Pose to a pose of type geometry_msgs::Pose, back to original type
    }

    double fraction=group_->computeCartesianPath(pose_goal, 0.01, 1.5, cart_path); // compute the cartesian path
    // part of ROS package, moveit_core, which is used to compute the cartesian path from the start pose to the goal pose
    // .01 is the step size, which is the distance between two adjacent points in the path
    // 1.5 is the jump threshold, which is the maximum distance between two adjacent points in the path

    if(fraction==-1) // if there is an error while computing the cartesian path
    {
        ROS_WARN("there is an error while computing the cartesian path");
        return;
    }

    if(fraction==1)
    {
        ROS_INFO("the cartesian path can be %.2f%% acheived", fraction * 100.0); // if the cartesian path can be achieved, print the percentage of the cartesian path that can be achieved
        trajectoryScaling(cart_path, velocity_scaling_); // scale the velocity of the cartesian path
        cart_plan.trajectory_=cart_path; // set the trajectory of the plan to the cartesian path
        group_->asyncExecute(cart_plan); // execute the plan asynchronously
    }
    else
    {
        ROS_INFO("the cartesian path can only be %.2f%% acheived and it will not be executed", fraction * 100.0);
    }
}

bool ElfinMotionAPI::getRefLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
// callback function for the service server, which is used to get the reference link
{
    resp.success=true;
    resp.message=reference_link_;
    return true;
}

bool ElfinMotionAPI::getEndLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    resp.success=true;
    resp.message=end_link_;
    return true;
}

void ElfinMotionAPI::torquesPubTimer_cb(const ros::TimerEvent& evt)
{
    std::vector<double> v_p; // v_p is a vector that stores the joint positions of the robot
    std::vector<double> v_v; // v_v is a vector that stores the joint velocities of the robot
    std::vector<double> v_a; // v_a is a vector that stores the joint accelerations of the robot
    std::vector<geometry_msgs::Wrench> v_w(7); // v_w is a vector that stores the wrenches of the robot
    // wrenches are forces and torques
    std::vector<double> v_t(6); // v_t is a vector that stores the torques of the robot
    for(int i=0;i<6; i++) // for each joint of the robot
    {
        v_v.push_back(0); // set the velocity of the joint to 0
        v_a.push_back(0); // set the acceleration of the joint to 0
    }

    robot_state::RobotStatePtr current_state=group_->getCurrentState(); // get the current state of the robot
    // example: the current state of the robot is [0, 0, 0, 0, 0, 0], which means the robot is at the home position
    const robot_state::JointModelGroup* jmp=current_state->getJointModelGroup(group_->getName());
    // get the joint model group of the robot, which is the group of joints that can be controlled
    // example: the joint model group of the robot is [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], which means the robot has 6 joints
    current_state->copyJointGroupPositions(jmp, v_p); // copy the joint positions of the robot to v_p

    dynamics_solver_->getTorques(v_p, v_v, v_a, v_w, v_t); // compute the torques of the robot

    torques_msg_.data=v_t; // set the data of the torques message to the torques of the robot

    torques_publisher_.publish(torques_msg_); // publish the torques of the robot
}

void ElfinMotionAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
}

void ElfinMotionAPI::setRefFrames(std::string ref_link)
{
    reference_link_=ref_link;
}

void ElfinMotionAPI::setEndFrames(std::string end_link)
{
    end_link_=end_link;
}

bool ElfinMotionAPI::updateTransforms(std::string ref_link)
{
  ros::Rate r(100); // create a ros rate object, which is used to specify the frequency of the loop
  int counter=0; // counter is used to count the number of times the loop has been executed
  while(ros::ok())
  {
      try{
        tf_listener_.waitForTransform(ref_link, root_link_, ros::Time(0), ros::Duration(10.0) ); // wait for the transform between the reference link and the root link
        // duration is the maximum time to wait for the transform
        tf_listener_.lookupTransform(ref_link, root_link_, ros::Time(0), transform_rootToRef_);
        // lookupTransform is a function of the tf::Transformer class, which is used to get the transform between the reference link and the root link
        break;
      }
      catch (tf::TransformException &ex) { // if there is an error while getting the transform
        r.sleep(); // sleep for 0.01 seconds
        counter++; // increment the counter
        if(counter>200) //if the counter is greater than 200
        {
          ROS_ERROR("%s",ex.what()); // print the error message
          ROS_ERROR("Motion planning failed");
          return false;
        }
        continue;
      }
  }

  counter=0;
  while(ros::ok())
  {
      try{
        tf_listener_.waitForTransform(end_link_, default_tip_link_, ros::Time(0), ros::Duration(10.0) );
        // wait for the transform between the end link and the tip link
        tf_listener_.lookupTransform(end_link_, default_tip_link_, ros::Time(0), transform_tipToEnd_);
        break;
      } // executiong similar to the previous loop
      catch (tf::TransformException &ex) {
        r.sleep();
        counter++;
        if(counter>200)
        {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("Motion planning failed");
          return false;
        }
        continue;
      }
  }

  return true;
}

}

