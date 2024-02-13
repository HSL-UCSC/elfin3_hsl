#include <low_level_control/low_level_control.h>

namespace low_level_control {
    // Constructor
    LowLevelControl::LowLevelControl(ros::NodeHandle &nh) : nh_(nh) {
        // Initialize member variables
        joint_cmd_topic_ = "/elfin/low_level/joint_cmd";
        stop_topic_ = "/elfin/low_level/stop";
        joint_move_srv_name_ = "/elfin/low_level/joint_move";
        stop_srv_name_ = "/elfin/low_level/stop";
        set_velocity_scaling_srv_name_ = "/elfin/low_level/set_velocity_scaling";
        set_ref_frames_srv_name_ = "/elfin/low_level/set_ref_frames";
        set_end_frames_srv_name_ = "/elfin/low_level/set_end_frames";
        joint_cmd_flag_ = false;
        stop_flag_ = false;
        joint_cmd_data_ = 0;
        joint_step_ = 0.01;
        velocity_scaling_ = 0.1;

        // Initialize ROS publishers, subscribers, and service clients
        joint_cmd_pub_ = nh_.advertise<std_msgs::Int64>(joint_cmd_topic_, 1);
        stop_pub_ = nh_.advertise<std_msgs::Empty>(stop_topic_, 1);
        joint_move_srv_ = nh_.advertiseService(joint_move_srv_name_, &LowLevelControl::jointMove_cb, this);
        stop_srv_ = nh_.advertiseService(stop_srv_name_, &LowLevelControl::stop_cb, this);
        set_velocity_scaling_srv_ = nh_.serviceClient<elfin_robot_msgs::SetFloat64>(set_velocity_scaling_srv_name_);
        set_ref_frames_srv_ = nh_.serviceClient<elfin_robot_msgs::SetInt16>(set_ref_frames_srv_name_);
        set_end_frames_srv_ = nh_.serviceClient<elfin_robot_msgs::SetInt16>(set_end_frames_srv_name_);
        joint_cmd_sub_ = nh_.subscribe("/elfin/low_level/joint_cmd", 1, &LowLevelControl::JointCmdCB, this);
        stop_sub_ = nh_.subscribe("/elfin/low_level/stop", 1, &LowLevelControl::StopCB, this);
    }

    // Destructor
    LowLevelControl::~LowLevelControl() {
    }

    // Callback function for receiving joint command
    void LowLevelControl::JointCmdCB(const std_msgs::Int64ConstPtr &msg) {
        joint_cmd_data_ = msg->data;
        joint_cmd_flag_ = true;
    }

    // Callback function for receiving stop command
    void LowLevelControl::StopCB(const std_msgs::EmptyConstPtr &msg) {
        stop_flag_ = true;
    }

    // Set velocity scaling
    void LowLevelControl::setVelocityScaling(double data) {
        elfin_robot_msgs::SetFloat64 srv;
        srv.request.data = data;
        set_velocity_scaling_srv_.call(srv);
    }

    // Set reference frames
    void LowLevelControl::setRefFrames(std::string ref_link) {
        elfin_robot_msgs::SetInt16 srv;
        srv.request.data = 0;
        set_ref_frames_srv_.call(srv);
    }

    // Set end frames
    void LowLevelControl::setEndFrames(std::string end_link) {
        elfin_robot_msgs::SetInt16 srv;
        srv.request.data = 0;
        set_end_frames_srv_.call(srv);
    }

    // Callback function for joint move service
    bool LowLevelControl::jointMove_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp) {
        // Check if the requested joint index is valid
        if (req.data == 0 || abs(req.data) > goal_.trajectory.joint_names.size()) {
            resp.success = false;
            resp.message = "Invalid joint index";
            return true;
        }

        // Get the joint index and current joint position
        int joint_nums = abs(req.data);
        std::vector<double> position_current = group_->getCurrentJointValues();
        std::vector<double> position_goal = position_current;
        double joint_current_position = position_current[joint_nums - 1];
        std::string direction = goal_.trajectory.joint_names[joint_nums - 1];
        double sign;

        // Determine the goal position and direction based on the requested joint index
        if (joint_num == req.data) {
            position_goal[joint_num - 1] = group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num - 1])->limits->upper;
            direction.append("+");
            sign = 1;
        } else {
            position_goal[joint_num - 1] = group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num - 1])->limits->lower;
            direction.append("-");
            sign = -1;
        }

        // Calculate the duration based on the joint speed
        double duration_from_speed = fabs(position_goal[joint_num - 1] - joint_current_position) / joint_speed_;

        // Check if the duration is too short for the robot to move
        if (duration_from_speed <= 0.1) {
            resp.success = false;
            std::string result = "Robot can't move to ";
            result.append(direction);
            result.append(" direction any more");
            resp.message = result;
            return true;
        }

        // Create a temporary trajectory point
        trajectory_msgs::JointTrajectoryPoint point_tmp;

        // Get the current kinematic state
        robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
        robot_state::RobotState kinematic_state = *kinematic_state_ptr;
        const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

        // Update the planning scene
        planning_scene_monitor_->updateFrameTransforms();
        planning_scene::PlanningSceneConstPtr plan_scene = planning_scene_monitor_->getPlanningScene();

        // Initialize temporary variables
        std::vector<double> position_tmp = position_current;
        bool collision_flag = false;
        int loop_num = 1;

        // Move the joint in small increments until the goal position is reached
        while (fabs(position_goal[joint_num - 1] - position_tmp[joint_num - 1]) / joint_speed_ > 0.1) {
            position_tmp[joint_num - 1] += joint_speed_ * 0.1 * sign;

            // Check for collisions
            kinematic_state.setJointGroupPositions(joint_model_group, position_tmp);
            if (plan_scene->isStateColliding(kinematic_state, group_->getName())) {
                if (loop_num == 1) {
                    resp.success = false;
                    std::string result = "Robot can't move in ";
                    result.append(direction);
                    result.append(" direction any more");
                    resp.message = result;
                    return true;
                }
                collision_flag = true;
                break;
            }

            // Add the trajectory point
            point_tmp.time_from_start = ros::Duration(0.1 * loop_num);
            point_tmp.positions = position_tmp;
            goal_.trajectory.points.push_back(point_tmp);
            loop_num++;
        }

        // Check for collisions and add the final trajectory point
        if (!collision_flag) {
            kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
            if (!plan_scene->isStateColliding(kinematic_state, group_->getName())) {
                point_tmp.positions = position_goal;
                ros::Duration dur(duration_from_speed);
                point_tmp.time_from_start = dur;
                goal_.trajectory.points.push_back(point_tmp);
            }
        }

        // Send the goal to the action client
        action_client_.sendGoal(goal_);
        goal_.trajectory.points.clear();

        resp.success = true;
        std::string result = "Robot is moving in ";
        result.append(direction);
        result.append(" direction");
        resp.message = result;
        return true;
    }

    // Callback function for stop service
    bool LowLevelControl::stop_cb(std::srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
        // Publish stop message
        std_msgs::Empty msg;
        stop_pub_.publish(msg);
        return true;
    }

    // Rotate the pose stamped message
    void LowLevelControl::PoseStampedRotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle) {
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose_stamped.pose.orientation, q);
        tf::Quaternion q_rot;
        q_rot.setRotation(axis, angle);
        q = q * q_rot;
        tf::quaternionTFToMsg(q, pose_stamped.pose.orientation);
    }
}