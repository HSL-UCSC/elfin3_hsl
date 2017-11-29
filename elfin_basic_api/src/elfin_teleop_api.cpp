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
ElfinTeleopAPI::ElfinTeleopAPI(moveit::planning_interface::MoveGroup *group, std::string action_name):
    group_(group), action_client_(action_name, true), teleop_nh_("~")
{
    goal_.trajectory.joint_names=group_->getJointNames();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nsec=0;
    sub_teleop_joint_command_no_limit_=root_nh_.subscribe("elfin_teleop_joint_cmd_no_limit", 1,  &ElfinTeleopAPI::teleopJointCmdNoLimitCB, this);

    joint_teleop_server_=teleop_nh_.advertiseService("joint_teleop", &ElfinTeleopAPI::jointTeleop_cb, this);
    cart_teleop_server_=teleop_nh_.advertiseService("cart_teleop", &ElfinTeleopAPI::cartTeleop_cb, this);
    home_teleop_server_=teleop_nh_.advertiseService("home_teleop", &ElfinTeleopAPI::homeTeleop_cb, this);
    teleop_stop_server_=teleop_nh_.advertiseService("stop_teleop", &ElfinTeleopAPI::teleopStop_cb, this);

    joint_step_=M_PI/100;
    joint_duration_ns_=1e+8;
    joint_speed_=0.2;
    cart_duration_=0.1;

    teleop_link_=group_->getEndEffectorLink();
}

void ElfinTeleopAPI::teleopJointCmdNoLimitCB(const std_msgs::Int64ConstPtr &msg)
{
    if(msg->data==0 || abs(msg->data)>goal_.trajectory.joint_names.size())
        return;
    int joint_num=abs(msg->data);
    int symbol;
    if(joint_num==msg->data)
        symbol=1;
    else
        symbol=-1;
    trajectory_msgs::JointTrajectoryPoint point_tmp;
    std::vector<double> position_tmp=group_->getCurrentJointValues();
    position_tmp[joint_num-1]+=symbol*joint_step_;
    point_tmp.positions=position_tmp;
    point_tmp.time_from_start.nsec=joint_duration_ns_;
    goal_.trajectory.points.push_back(point_tmp);
    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();
}

bool ElfinTeleopAPI::jointTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>goal_.trajectory.joint_names.size())
    {
        resp.success=false;
        resp.message="wrong joint teleop data";
        return true;
    }
    int joint_num=abs(req.data);
    trajectory_msgs::JointTrajectoryPoint point_tmp;
    std::vector<double> position_tmp=group_->getCurrentJointValues();
    double joint_current_position=position_tmp[joint_num-1];
    std::string direction=goal_.trajectory.joint_names[joint_num-1];
    if(joint_num==req.data)
    {
        position_tmp[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->upper;
        direction.append("+");
    }
    else
    {
        position_tmp[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->lower;
        direction.append("-");
    }

    point_tmp.positions=position_tmp;
    double duration_from_speed=fabs(position_tmp[joint_num-1]-joint_current_position)/joint_speed_;
    if(duration_from_speed<0.1)
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }
    ros::Duration dur(duration_from_speed);
    point_tmp.time_from_start=dur;
    goal_.trajectory.points.push_back(point_tmp);
    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;
}

bool ElfinTeleopAPI::cartTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>6)
    {
        resp.success=false;
        resp.message="wrong cart. teleop data";
        return true;
    }
    int operation_num=abs(req.data);
    int symbol;
    if(operation_num==req.data)
        symbol=1;
    else
        symbol=-1;

    geometry_msgs::PoseStamped current_pose=group_->getCurrentPose(teleop_link_);
    std::vector<double> current_joint_states=group_->getCurrentJointValues();

    tf::Vector3 x_axis(1, 0, 0);
    tf::Vector3 y_axis(0, 1, 0);
    tf::Vector3 z_axis(0, 0, 1);
    double resolution_alpha=0.02;
    double resolution_delta=0.005;

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    robot_state::RobotState kinematic_state=*kinematic_state_ptr;
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    trajectory_msgs::JointTrajectoryPoint point_tmp;

    std::string direction;

    bool ik_have_result=true;
    for(int i=0; i<100; i++)
    {
        switch (operation_num) {
        case 1:
            current_pose.pose.position.x+=symbol*resolution_delta;
            if(symbol==1)
                direction="X+";
            else
                direction="X-";
            break;
        case 2:
            current_pose.pose.position.y+=symbol*resolution_delta;
            if(symbol==1)
                direction="Y+";
            else
                direction="Y-";
            break;
        case 3:
            current_pose.pose.position.z+=symbol*resolution_delta;
            if(symbol==1)
                direction="Z+";
            else
                direction="Z-";
            break;
        case 4:
            PoseStampedRotation(current_pose, x_axis, symbol*resolution_alpha);
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
        ik_have_result=kinematic_state.setFromIK(joint_model_group, current_pose.pose, teleop_link_);
        if(ik_have_result)
        {
            if(goal_.trajectory.points.size()!=i)
                break;
            point_tmp.positions.resize(goal_.trajectory.joint_names.size());
            double biggest_shift=0;
            for(int j=0; j<goal_.trajectory.joint_names.size(); j++)
            {
                point_tmp.positions[j]=*kinematic_state.getJointPositions(goal_.trajectory.joint_names[j]);
                if(i==0)
                {
                    double shift_tmp=fabs(current_joint_states[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
                else {
                    double shift_tmp=fabs(goal_.trajectory.points[i-1].positions[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
            }
            if(biggest_shift>cart_duration_*1)
                break;
            ros::Duration dur((i+1)*cart_duration_);
            point_tmp.time_from_start=dur;
            goal_.trajectory.points.push_back(point_tmp);
        }
        else
        {
            break;
        }
    }
    if(goal_.trajectory.points.size()==0)
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }
    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;

}

bool ElfinTeleopAPI::homeTeleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    trajectory_msgs::JointTrajectoryPoint point_tmp;
    std::vector<double> position_tmp=group_->getCurrentJointValues();
    std::vector<double> position_goal;
    double biggest_shift=0;

    if(position_tmp.size()!=goal_.trajectory.joint_names.size())
    {
        resp.success=false;
        resp.message="the joints number is wrong";
        return true;
    }

    for(int i=0; i<position_tmp.size(); i++)
    {
        if(fabs(position_tmp[i])>biggest_shift)
        {
            biggest_shift=fabs(position_tmp[i]);
        }
        position_goal.push_back(0);
    }

    double duration_from_speed=biggest_shift/joint_speed_;
    ros::Duration dur(duration_from_speed);
    point_tmp.time_from_start=dur;
    point_tmp.positions=position_goal;
    goal_.trajectory.points.push_back(point_tmp);
    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();

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
{
    tf::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                 pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    tf::Quaternion q_2(axis, angle);
    tf::Matrix3x3 m(q_1);
    tf::Matrix3x3 m_2(q_2);
//    m.operator *=(m_2);
    m_2.operator *=(m);
    double r, p, y;
    m_2.getRPY(r,p,y);

    q_2.setRPY(r, p, y);
    pose_stamped.pose.orientation.x=q_2.getX();
    pose_stamped.pose.orientation.y=q_2.getY();
    pose_stamped.pose.orientation.z=q_2.getZ();
    pose_stamped.pose.orientation.w=q_2.getW();
}

} // end namespace elfin_basic_api