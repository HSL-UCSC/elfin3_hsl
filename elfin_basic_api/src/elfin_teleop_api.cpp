/*
Created on Mon Nov 13 15:20:10 2017

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

#include "elfin_basic_api/elfin_teleop_api.h"

namespace elfin_basic_api {
ElfinTeleopAPI::ElfinTeleopAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
    group_(group), action_client_(action_name, true), planning_scene_monitor_(planning_scene_monitor), teleop_nh_("~")
{
    goal_.trajectory.joint_names=group_->getJointNames();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nsec=0;
    sub_teleop_joint_command_no_limit_=root_nh_.subscribe("elfin_teleop_joint_cmd_no_limit", 1,  &ElfinTeleopAPI::teleopJointCmdNoLimitCB, this);

    joint_teleop_server_=teleop_nh_.advertiseService("joint_teleop", &ElfinTeleopAPI::jointTeleop_cb, this);
    cart_teleop_server_=teleop_nh_.advertiseService("cart_teleop", &ElfinTeleopAPI::cartTeleop_cb, this);
    home_teleop_server_=teleop_nh_.advertiseService("home_teleop", &ElfinTeleopAPI::homeTeleop_cb, this);
    teleop_stop_server_=teleop_nh_.advertiseService("stop_teleop", &ElfinTeleopAPI::teleopStop_cb, this);

    // parameter for teleop no limit
    joint_step_=M_PI/100; // 0.01 rad
    joint_duration_ns_=1e+8; // 0.1 sec

    // parameter for normal teleop
    joint_speed_limit_=JOINT_SPEED_LIMIT_CONST; // 0.5 rad/sec
    joint_speed_default_=JOINT_SPEED_DEFAULT_CONST; // 0.1 rad/sec
    cart_duration_default_=CART_DURATION_DEFAULT_CONST; // 0.5 sec
    resolution_angle_=0.02; // 0.02 rad
    resolution_linear_=0.005; // 0.005 m

    end_link_=group_->getEndEffectorLink();
    reference_link_=group_->getPlanningFrame();

    default_tip_link_=group_->getEndEffectorLink();
    root_link_=group_->getPlanningFrame();
}

void ElfinTeleopAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data; // data is in the range of [0, 1]
    joint_speed_=joint_speed_default_*velocity_scaling_; // joint_speed_ is in the range of [0, joint_speed_limit_]
    // example: if joint_speed_limit_=0.5, joint_speed_default_=0.1, velocity_scaling_=0.5, then joint_speed_=0.05
    cart_duration_=cart_duration_default_/velocity_scaling_; // cart_duration_ is in the range of [0, 2*cart_duration_default_]
    // example: if cart_duration_default_=0.5, velocity_scaling_=0.5, then cart_duration_=1
    group_->setMaxVelocityScalingFactor(velocity_scaling_);
}

void ElfinTeleopAPI::setRefFrames(std::string ref_link)
{
    reference_link_=ref_link;
}

void ElfinTeleopAPI::setEndFrames(std::string end_link)
{
    end_link_=end_link;
}

void ElfinTeleopAPI::teleopJointCmdNoLimitCB(const std_msgs::Int64ConstPtr &msg)
{
    if(msg->data==0 || abs(msg->data)>goal_.trajectory.joint_names.size()) // if the data is 0 or the data is out of the range of [1, joint_names.size()]
        return;
    int joint_num=abs(msg->data); // joint_num is the number of the joint
    int symbol; // symbol is the direction of the joint
    if(joint_num==msg->data) // if joint_num is positive
        symbol=1; // symbol is 1
    else
        symbol=-1; // symbol is -1
    trajectory_msgs::JointTrajectoryPoint point_tmp; // create a joint trajectory point
    std::vector<double> position_tmp=group_->getCurrentJointValues(); // get the current joint values of the robot
    position_tmp[joint_num-1]+=symbol*joint_step_; // add the joint_step_ to the joint_num-th joint value
    point_tmp.positions=position_tmp; // set the joint values of the joint trajectory point
    point_tmp.time_from_start.nsec=joint_duration_ns_; // set the duration of the joint trajectory point
    goal_.trajectory.points.push_back(point_tmp); // add the joint trajectory point to the goal
    action_client_.sendGoal(goal_); // send the goal to the action server
    goal_.trajectory.points.clear(); // clear the goal
}

bool ElfinTeleopAPI::jointTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>goal_.trajectory.joint_names.size()) // if the data is 0 or the data is out of the range of [1, joint_names.size()]
    {
        resp.success=false;
        resp.message="wrong joint teleop data";
        return true;
    }
    int joint_num=abs(req.data); // joint_num is the number of the joint
    std::vector<double> position_current=group_->getCurrentJointValues(); // get the current joint values of the robot
    std::vector<double> position_goal=position_current; // position_goal is the goal joint values of the robot
    double joint_current_position=position_current[joint_num-1]; // joint_current_position is the current position of the joint
    std::string direction=goal_.trajectory.joint_names[joint_num-1]; // direction is the name of the joint
    double sign; // sign is the direction of the joint
    if(joint_num==req.data) // if joint_num is equal to the data
    {
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->upper;
        // position_goal[joint_num-1] is the upper limit of the joint
        // getRobotModel() is used to get the robot model, which gets the joint limits from file "elfin_robot_description/urdf/elfin3.urdf.xacro"
        direction.append("+"); // direction is the name of the joint plus "+"
        sign=1; // positive
    }
    else
    {
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->lower;
        // position_goal[joint_num-1] is the lower limit of the joint
        direction.append("-");
        sign=-1; // negative
    }

    double duration_from_speed=fabs(position_goal[joint_num-1]-joint_current_position)/joint_speed_;
    // duration_from_speed is the duration of the joint trajectory point
    // fabs() is used to get the absolute value of a number
    // example: if joint_speed_=0.5, joint_current_position=0, position_goal[joint_num-1]=1, then duration_from_speed=2
    if(duration_from_speed<=0.1) // if duration_from_speed is less than or equal to 0.1
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }

    trajectory_msgs::JointTrajectoryPoint point_tmp; // create a joint trajectory point

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState(); // get the current state of the robot
    robot_state::RobotState kinematic_state=*kinematic_state_ptr; // kinematic_state is the current state of the robot
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
    // joint_model_group is the joint model group of the robot
    // kinematic state is used to get the joint values of the robot

    planning_scene_monitor_->updateFrameTransforms(); // update the transforms between the reference link and the root link, and between the end link and the tip link
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();
    // planning_scene is used to get the current state of the robot

    std::vector<double> position_tmp=position_current; // position_tmp is the temporary joint values of the robot
    bool collision_flag=false;

    int loop_num=1;
    while(fabs(position_goal[joint_num-1]-position_tmp[joint_num-1])/joint_speed_>0.1)
    // if the distance between the goal joint value and the temporary joint value is greater than 0.1
    {
        position_tmp[joint_num-1]+=joint_speed_*0.1*sign; // add the joint_speed_*0.1*sign to the temporary joint value

        kinematic_state.setJointGroupPositions(joint_model_group, position_tmp); // set the joint values of the robot
        if(plan_scene->isStateColliding(kinematic_state, group_->getName())) // if the robot is in collision
        {
            if(loop_num==1) // if loop_num is 1
            {
                resp.success=false;
                std::string result="robot can't move in ";
                result.append(direction);
                result.append(" direction any more");
                resp.message=result;
                return true;
            }
            collision_flag=true;
            break;
        }

        point_tmp.time_from_start=ros::Duration(0.1*loop_num); // set the duration of the joint trajectory point
        point_tmp.positions=position_tmp; // set the joint values of the joint trajectory point
        goal_.trajectory.points.push_back(point_tmp); // add the joint trajectory point to the goal
        loop_num++;
    }

    if(!collision_flag)
    {
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal); // set the joint values of the robot
        if(!plan_scene->isStateColliding(kinematic_state, group_->getName())) // if the robot is not in collision
        {
            point_tmp.positions=position_goal; // set the joint values of the joint trajectory point
            ros::Duration dur(duration_from_speed); // set the duration of the joint trajectory point
            point_tmp.time_from_start=dur; // set the duration of the joint trajectory point
            goal_.trajectory.points.push_back(point_tmp); // add the joint trajectory point to the goal
        }
    }

    action_client_.sendGoal(goal_); // send the goal to the action server, where the robot will move to the goal
    goal_.trajectory.points.clear(); // clear the goal

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;
}

bool ElfinTeleopAPI::cartTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp)
// cartTeleop_cb is the callback function for the service server, which is used to control the robot to a cartesian goal
{
    if(req.data==0 || abs(req.data)>6) // if the data is 0 or the data is out of the range of [1, 6]
    {
        resp.success=false;
        resp.message="wrong cart. teleop data";
        return true;
    }
    int operation_num=abs(req.data); // operation_num is the number of the operation
    int symbol; // symbol is the direction of the operation
    if(operation_num==req.data) // if operation_num is equal to the data
        symbol=1;
    else
        symbol=-1;

    geometry_msgs::PoseStamped current_pose=group_->getCurrentPose(end_link_); // get the current pose of the robot

    ros::Rate r(100); // r is the rate of the timer
    int counter=0;
    while(ros::ok()) 
    {
        try{
          tf_listener_.waitForTransform(reference_link_, root_link_, ros::Time(0), ros::Duration(10.0) );
          tf_listener_.lookupTransform(reference_link_, root_link_, ros::Time(0), transform_rootToRef_);
          break;
        }
        catch (tf::TransformException &ex) {
          r.sleep();
          counter++;
          if(counter>200)
          {
            ROS_ERROR("%s",ex.what());
            resp.success=false;
            resp.message="can't get pose of reference frame";
            return true;
          }
          continue;
        }
    }

    counter=0;
    while(ros::ok())
    {
        try{
          tf_listener_.waitForTransform(end_link_, default_tip_link_, ros::Time(0), ros::Duration(10.0) );
          tf_listener_.lookupTransform(end_link_, default_tip_link_, ros::Time(0), transform_tipToEnd_);
          break;
        }
        catch (tf::TransformException &ex) {
          r.sleep();
          counter++;
          if(counter>200)
          {
            ROS_ERROR("%s",ex.what());
            resp.success=false;
            resp.message="can't get pose of teleop frame";
            return true;
          }
          continue;
        }
    }

    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    // affine_rootToRef is the transform between the reference link and the root link
    tf::transformTFToEigen(transform_rootToRef_, affine_rootToRef);
    // transformTFToEigen is used to transform the transform between the reference link and the root link to the transform between the reference link and the root link in Eigen
    affine_refToRoot=affine_rootToRef.inverse();
    // affine_refToRoot is the inverse of affine_rootToRef

    Eigen::Isometry3d affine_tipToEnd;
    // affine_tipToEnd is the transform between the end link and the tip link
    tf::transformTFToEigen(transform_tipToEnd_, affine_tipToEnd);
    // transformTFToEigen is used to transform the transform between the end link and the tip link to the transform between the end link and the tip link in Eigen

    tf::Pose tf_pose_tmp;
    // tf_pose_tmp is the pose of the robot in tf
    Eigen::Isometry3d affine_pose_tmp;
    // affine_pose_tmp is the pose of the robot in Eigen
    tf::poseMsgToTF(current_pose.pose, tf_pose_tmp);
    // poseMsgToTF is used to transform the pose of the robot to the pose of the robot in tf
    tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp);
    // poseTFToEigen is used to transform the pose of the robot to the pose of the robot in Eigen

    Eigen::Isometry3d affine_current_pose=affine_rootToRef * affine_pose_tmp;
    // affine_current_pose is the current pose of the robot in Eigen

    tf::poseEigenToTF(affine_current_pose, tf_pose_tmp);
    // poseEigenToTF is used to transform the pose of the robot to the pose of the robot in tf

    tf::poseTFToMsg(tf_pose_tmp, current_pose.pose);
    // poseTFToMsg is used to transform the pose of the robot to the pose of the robot in msg

    std::vector<double> current_joint_states=group_->getCurrentJointValues();
    // current_joint_states is the current joint values of the robot

    tf::Vector3 x_axis(1, 0, 0);
    // x_axis is the x axis of the robot
    tf::Vector3 y_axis(0, 1, 0); // y_axis is the y axis of the robot
    tf::Vector3 z_axis(0, 0, 1); // z_axis is the z axis of the robot
    double resolution_alpha=resolution_angle_; // resolution_alpha is the resolution of the rotation
    double resolution_delta=resolution_linear_; // resolution_delta is the resolution of the translation

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState(); // get the current state of the robot
    robot_state::RobotState kinematic_state=*kinematic_state_ptr; // kinematic_state is the current state of the robot
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
    // joint_model_group is the joint model group of the robot which is set to the joint model group of the robot

    planning_scene_monitor_->updateFrameTransforms();
    // update the transforms between the reference link and the root link, and between the end link and the tip link
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();
    // planning_scene is used to get the current state of the robot

    trajectory_msgs::JointTrajectoryPoint point_tmp;
    // point_tmp is the joint trajectory point

    std::string direction; // direction is the direction of the operation

    bool ik_have_result=true; // ik_have_result is used to check if the robot can move to the goal
    for(int i=0; i<100; i++) // i is the number of the joint trajectory point
    {
        switch (operation_num) { // operation_num is the number of the operation
        case 1:
            current_pose.pose.position.x+=symbol*resolution_delta; // add the resolution_delta to the x position of the pose
            if(symbol==1)
                direction="X+"; 
            else
                direction="X-"; 
            break;
        case 2:
            current_pose.pose.position.y+=symbol*resolution_delta; // add the resolution_delta to the y position of the pose
            if(symbol==1)
                direction="Y+"; 
            else
                direction="Y-";
            break;
        case 3:
            current_pose.pose.position.z+=symbol*resolution_delta; // add the resolution_delta to the z position of the pose
            if(symbol==1)
                direction="Z+";
            else
                direction="Z-";
            break;
        case 4:
            PoseStampedRotation(current_pose, x_axis, symbol*resolution_alpha); // rotate the pose around the x axis by the symbol*resolution_alpha
            if(symbol==1)
                direction="Rx+";
            else
                direction="Rx-";
            break;
        case 5:
            PoseStampedRotation(current_pose, y_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Ry+";
            else
                direction="Ry-";
            break;
        case 6:
            PoseStampedRotation(current_pose, z_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rz+";
            else
                direction="Rz-";
            break;
        default:
            break;
        }

        tf::poseMsgToTF(current_pose.pose, tf_pose_tmp); // transform the pose of the robot to the pose of the robot in tf
        tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp); // transform the pose of the robot to the pose of the robot in Eigen

        affine_current_pose=affine_refToRoot * affine_pose_tmp * affine_tipToEnd;
        // affine_current_pose is the current pose of the robot in Eigen

        ik_have_result=kinematic_state.setFromIK(joint_model_group, affine_current_pose, default_tip_link_);
        // setFromIK is used to set the joint values of the robot
        if(ik_have_result)
        {
            if(goal_.trajectory.points.size()!=i) // if the number of the joint trajectory point is not equal to i
                break;
            point_tmp.positions.resize(goal_.trajectory.joint_names.size()); // resize the joint values of the joint trajectory point
            double biggest_shift=0; // biggest_shift is the biggest shift of the joint values
            for(int j=0; j<goal_.trajectory.joint_names.size(); j++) // j is the number of joints
            {
                point_tmp.positions[j]=*kinematic_state.getJointPositions(goal_.trajectory.joint_names[j]); // set the joint values of the joint trajectory point
                if(i==0)
                {
                    double shift_tmp=fabs(current_joint_states[j]-point_tmp.positions[j]); // shift_tmp is the shift of the joint values
                    if(shift_tmp>biggest_shift) // if shift_tmp is greater than biggest_shift
                        biggest_shift=shift_tmp; // set biggest_shift to shift_tmp
                }
                else {
                    double shift_tmp=fabs(goal_.trajectory.points[i-1].positions[j]-point_tmp.positions[j]);
                    // shift_tmp is the shift of the joint values
                    if(shift_tmp>biggest_shift) // if shift_tmp is greater than biggest_shift
                        biggest_shift=shift_tmp; // set biggest_shift to shift_tmp
                }
            }
            if(biggest_shift>cart_duration_*joint_speed_limit_ || plan_scene->isStateColliding(kinematic_state, group_->getName()))
            // if biggest_shift is greater than cart_duration_*joint_speed_limit_ or the robot is in collision
                break;
            ros::Duration dur((i+1)*cart_duration_); // set the duration of the joint trajectory point
            point_tmp.time_from_start=dur; // set the duration of the joint trajectory point
            goal_.trajectory.points.push_back(point_tmp); // add the joint trajectory point to the goal
        }
        else
        {
            break;
        }
    }
    if(goal_.trajectory.points.size()==0) // if the number of the joint trajectory point is 0
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }
    action_client_.sendGoal(goal_); // send the goal to the action server, where the robot will move to the goal
    goal_.trajectory.points.clear(); // clear the goal

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;

}

bool ElfinTeleopAPI::homeTeleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
// homeTeleop_cb is the callback function for the service server, which is used to control the robot to the home position
{
    std::vector<double> position_current=group_->getCurrentJointValues();
    // position_current is the current joint values of the robot
    std::vector<double> position_goal;
    // position_goal is the goal joint values of the robot
    double biggest_shift=0;
    int biggest_shift_num;
    std::vector<double> joint_ratios;
    // joint_ratios is the ratios of the joint values
    joint_ratios.resize(position_current.size());
    // resize the joint values of the joint ratios

    if(position_current.size()!=goal_.trajectory.joint_names.size())
    // if the size of the current joint values is not equal to the size of the joint names
    {
        resp.success=false;
        resp.message="the joints number is wrong";
        return true;
    }

    for(int i=0; i<position_current.size(); i++) // i is the number of joints
    {
        if(fabs(position_current[i])>biggest_shift) // if the absolute value of the current joint value is greater than biggest_shift
        {
            biggest_shift=fabs(position_current[i]); // set biggest_shift to the absolute value of the current joint value
            biggest_shift_num=i; // set biggest_shift_num to i
        }
        position_goal.push_back(0); // add 0 to the goal joint values
    }

    if(biggest_shift<0.001) // if biggest_shift is less than 0.001
    {
        resp.success=false;
        std::string result="Elfin is already in home position";
        resp.message=result;
        return true;
    }

    for(int i=0; i<joint_ratios.size(); i++) // i is the number of joints
    {
        joint_ratios[i]=position_current[i]/position_current[biggest_shift_num]; // set the joint ratios
    }

    trajectory_msgs::JointTrajectoryPoint point_tmp; // create a joint trajectory point

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState(); // get the current state of the robot
    robot_state::RobotState kinematic_state=*kinematic_state_ptr; // kinematic_state is the current state of the robot
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
    // joint_model_group is the joint model group of the robot

    planning_scene_monitor_->updateFrameTransforms(); // update the transforms between the reference link and the root link, and between the end link and the tip link
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();
    // planning_scene is used to get the current state of the robot

    std::vector<double> position_tmp=position_current; // position_tmp is the temporary joint values of the robot
    bool collision_flag=false;

    int loop_num=1; // loop_num is the number of the joint trajectory point
    double sign=biggest_shift/position_tmp[biggest_shift_num]; // sign is the direction of the joint
    double duration_from_speed=biggest_shift/joint_speed_; // duration_from_speed is the duration of the joint trajectory point

    while(fabs(position_goal[biggest_shift_num]-position_tmp[biggest_shift_num])/joint_speed_>0.1)
    // if the distance between the goal joint value and the temporary joint value is greater than 0.1
    {
        for(int i=0; i<position_tmp.size(); i++) // i is the number of joints
        {
            position_tmp[i]-=joint_speed_*0.1*sign*joint_ratios[i]; // add the joint_speed_*0.1*sign*joint_ratios[i] to the temporary joint value
        }

        kinematic_state.setJointGroupPositions(joint_model_group, position_tmp); // set the joint values of the robot
        if(plan_scene->isStateColliding(kinematic_state, group_->getName())) // if the robot is in collision
        {
            if(loop_num==1)
            {
                resp.success=false;
                std::string result="Stop going to home position";
                resp.message=result;
                return true;
            }
            collision_flag=true;
            break;
        }

        point_tmp.time_from_start=ros::Duration(0.1*loop_num); // set the duration of the joint trajectory point
        point_tmp.positions=position_tmp; // set the joint values of the joint trajectory point
        goal_.trajectory.points.push_back(point_tmp); // add the joint trajectory point to the goal
        loop_num++;
    }

    if(!collision_flag)
    {
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
        // set the joint values of the robot
        if(!plan_scene->isStateColliding(kinematic_state, group_->getName()))
        // if the robot is not in collision
        {
            point_tmp.positions=position_goal;
            ros::Duration dur(duration_from_speed);
            point_tmp.time_from_start=dur;
            goal_.trajectory.points.push_back(point_tmp);
        }
        else if(loop_num==1) // if first iteration
        {
            resp.success=false;
            std::string result="Stop going to home position";
            resp.message=result;
            return true;
        }
    }

    action_client_.sendGoal(goal_); // send the goal to the action server, where the robot will move to the goal
    goal_.trajectory.points.clear(); // clear the goal

    resp.success=true;
    std::string result="robot is moving to home position";
    resp.message=result;
    return true;

}

bool ElfinTeleopAPI::teleopStop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    goal_.trajectory.points.clear();
    action_client_.sendGoal(goal_);

    resp.success=true;
    resp.message="stop moving";
    return true;
}

void ElfinTeleopAPI::PoseStampedRotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle)
// PoseStampedRotation is used to rotate the pose_stamped around the axis by the angle
{
    tf::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                 pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
                                 // q_1 is the quaternion of the pose_stamped
    tf::Quaternion q_2(axis, angle); // q_2 is the quaternion of the rotation
    tf::Matrix3x3 m(q_1); // m is the rotation matrix of the pose_stamped
    tf::Matrix3x3 m_2(q_2); // m_2 is the rotation matrix of the rotation
    m_2.operator *=(m); // m_2 is the rotation matrix of the pose_stamped after the rotation
    double r, p, y; // r is the roll, p is the pitch, y is the yaw
    m_2.getRPY(r,p,y); // get the roll, pitch and yaw of the pose_stamped after the rotation

    q_2.setRPY(r, p, y); // set the quaternion of the rotation
    pose_stamped.pose.orientation.x=q_2.getX(); // set the quaternion of the pose_stamped after the rotation
    pose_stamped.pose.orientation.y=q_2.getY(); 
    pose_stamped.pose.orientation.z=q_2.getZ(); 
    pose_stamped.pose.orientation.w=q_2.getW(); 
}

} // end namespace elfin_basic_api
