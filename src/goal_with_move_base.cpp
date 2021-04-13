// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <multi_control/RobotStatus.h>

// eigen includes
#include <Eigen/Eigen>
#include <string>

// Visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <string>

using namespace std;

// joint state subsciber to get current joint value
ros::Subscriber robot_state_sub_;
ros::Subscriber goal_status_sub_;
ros::Publisher traj_pub_;

Eigen::Vector4d current_state_, last_state_;
string robot_id;
double trajecty_sum_;
int flag;
bool flag_trajecty;

visualization_msgs::Marker trajectory_vis;

void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg);
void goalStatusCallBack(const multi_control::RobotStatus::ConstPtr& msg);
void publishTrajectory(void);

// read current position and velocity of robot joints
void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    //ROS_INFO("StateCallBack...");
    last_state_ = current_state_;

    current_state_(0) =    msg->position.x;
    current_state_(1) =    msg->position.y;
    current_state_(2) =    msg->orientation.z;
    current_state_(3) =    msg->position.z;
    if(flag_trajecty){
        trajecty_sum_ += std::sqrt(std::pow(current_state_(0) - last_state_(0),2)+std::pow(current_state_(1) - last_state_(1),2));
    }
    flag_trajecty = true;
    publishTrajectory();
}

void goalStatusCallBack(const multi_control::RobotStatus::ConstPtr& msg)
{
    ROS_INFO("goalStatusCallBack...");
    //int robot_id = msg->robot_id;
    bool ready = msg->is_ready;
    if(ready){
        ROS_INFO("Robot %s id get to goal!", robot_id.c_str());
        ROS_INFO("sum: %f !", trajecty_sum_);
    }
    else{
        ROS_INFO("going...");
    }
}

void publishTrajectory(void)
{
    //ROS_INFO("LMPCC::publishTrajectory");
    // Create MarkerArray for global path point visualization
    visualization_msgs::MarkerArray plan;

    trajectory_vis.id = 1000 + flag ;
    trajectory_vis.pose.position.x = current_state_(0);
    trajectory_vis.pose.position.y = current_state_(1);
    trajectory_vis.pose.orientation.x = 0;
    trajectory_vis.pose.orientation.y = 0;
    trajectory_vis.pose.orientation.z = 0;
    trajectory_vis.pose.orientation.w = 1;
    plan.markers.push_back(trajectory_vis);

    flag++;
    // Publish markerarray of global path points
    traj_pub_.publish(plan);
}

int main(int argc, char** argv)
{
    if(argc < 2){
        ROS_ERROR("You must specify robot id.");
        return -1;
    }

    char *id = argv[1];
    robot_id = id;

    flag = 0;
    flag_trajecty = false;
    float color_r_ = 0.0;
    float color_g_ = 0.0;
    float color_b_ = 0.0;
    trajecty_sum_ = 0.0;

    ROS_INFO("robot no. %s", robot_id.c_str());

    ros::init(argc, argv, "state_monitor");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    if (!n.getParam ("color_r", color_r_) )
    {
        ROS_WARN(" Parameter 'color_r not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }
    if (!n.getParam ("color_g", color_g_) )
    {
        ROS_WARN(" Parameter 'color_r not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }
    if (!n.getParam ("color_b", color_b_) )
    {
        ROS_WARN(" Parameter 'color_r not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // color_r_ = nh.getParam("color_r", 0.8);
    // color_g_ = nh.getParam("color_g", 0.0);
    // color_b_ = nh.getParam("color_b", 0.0);

    robot_state_sub_ = nh.subscribe("robot_state", 1, &StateCallBack);
    goal_status_sub_ = nh.subscribe("goal_status", 1, &goalStatusCallBack);
    traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory", 1);

    trajectory_vis.type = visualization_msgs::Marker::CYLINDER;
    trajectory_vis.id = 1000;
    trajectory_vis.color.r = color_r_;
    trajectory_vis.color.g = color_g_;
    trajectory_vis.color.b = color_b_;
    trajectory_vis.color.a = 0.8;
    trajectory_vis.header.frame_id = "map";
    trajectory_vis.ns = "trajectory_test";
    trajectory_vis.action = visualization_msgs::Marker::ADD;
    trajectory_vis.lifetime = ros::Duration(0);
    trajectory_vis.scale.x = 0.05;
    trajectory_vis.scale.y = 0.05;
    trajectory_vis.scale.z = 0.05;

    ros::spin();
}