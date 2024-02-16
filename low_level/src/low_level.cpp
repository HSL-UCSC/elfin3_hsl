#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include 

class low_level {

    private:
    int counter;
    // ros::Publisher pub;
    ros::Subscriber joint_states_subscriber;
    ros::ServiceServer desired_joint_states_service;

    public:
    low_level(ros::NodeHandle *nh) {
        joint = [5];

        joint_states_subscriber = nh->subscribe("/joint_states", 1000, 
            &low_level::callback_joint_states, this);
        desired_joint_states_service = nh->advertiseService("/desired_joint_states",
            &low_level::callback_desired_joint_states, this);
    }

    // NumberCounter(ros::NodeHandle *nh) {
    //     counter = 0;

    //     pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
    //     number_subscriber = nh->subscribe("/number", 1000, 
    //         &NumberCounter::callback_number, this);
    //     reset_service = nh->advertiseService("/reset_counter", 
    //         &NumberCounter::callback_reset_counter, this);
    // }

    void callback_number(const std_msgs::Int64& msg) {
        // take joint states data and store in private variables
        joint[0] = msg.data[0];
        joint[1] = msg.data[1];
        joint[2] = msg.data[2];
        joint[3] = msg.data[3];
        joint[4] = msg.data[4];
        joint[5] = msg.data[5];
    }

    bool callback_reset_counter(std_srvs::SetBool::Request &req, 
                                std_srvs::SetBool::Response &res)
    {
        /* if (req.data) {
            if (joint == [0, 0, 0, 0, 0, 0]) {
                res.success = false;
                res.message = "Joint states have not been set";
            }
            else {
                res.success = true;
                res.message = "Joint states have been successfully set";
            }
        } */

        int joint_num = abs(msg->data);
        double current_joint_values[6] = group.getCurrentJointValues();
        int current_pos = current_joint_values[joint_num] * 180 / 3.14159;
        // take desired position and move requested joint to that position
        int direction;
        while (joint[joint_num] - msg->data > 0.1) {
            if (joint_num == msg->data) {
                direction = 1;
            }
            else {
                direction = -1;
            }
        }
        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "low_level");
    ros::NodeHandle nh;
    NumberCounter nc = NumberCounter(&nh);
    ros::spin();
}