#pragma once
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#define CVUI_IMPLEMENTATION

// Files 
#include "robot_gui/cvui.h"

// Others
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// Messages
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"

class RobotGui{

    public:
        RobotGui(ros::NodeHandle *node_handle);
        RobotGui();
        void run();
        void general_info_area_init();
        void teleoperation_buttons();
        void odometry();

    private:

        // GENERAL INFO 
        ros::Subscriber sub_;
        std_msgs::Float64 data;
        std::string topic_name_sub;
        void msgCallback(const std_msgs::Float64::ConstPtr &msg);
        
        // WINDOW NAME
        const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";

        // CMD VEL PUBLISH
        ros::Publisher twist_pub_;
        geometry_msgs::Twist twist_msg;
        std::string twist_topic_name;
        float linear_velocity_step = 0.1;
        float angular_velocity_step = 0.1;

        // ODOMETRY
        ros::Subscriber odom_sub_;
        nav_msgs::Odometry odom_data;
        std::string odom_topic_name;
        void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
        
    protected:
        ros::NodeHandle *nh;
        
    

};

