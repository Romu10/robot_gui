#include "nav_msgs/Odometry.h"
#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "std_msgs/Float64.h"

RobotGui::RobotGui(ros::NodeHandle *node_handle){

    nh = node_handle;
    general_info_area_init();
    teleoperation_buttons();
    odometry();

}

void RobotGui::general_info_area_init(){
    topic_name_sub = "/robot_info";
    sub_ = nh->subscribe<std_msgs::Float64>(topic_name_sub, 2,
                                          &RobotGui::msgCallback, this);
}

void RobotGui::teleoperation_buttons(){
    twist_topic_name = "cmd_vel";
    twist_pub_ = nh->advertise<geometry_msgs::Twist>(twist_topic_name, 10);
}

void RobotGui::odometry(){
    odom_topic_name = "/odom";
    sub_ = nh->subscribe<nav_msgs::Odometry>(
            odom_topic_name, 2, &RobotGui::OdomMsgCallback, this);
}

void RobotGui::msgCallback(const std_msgs::Float64::ConstPtr &msg){
    data = *msg;
    ROS_DEBUG("Number received: %f", msg->data);
}

void RobotGui::OdomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_data = *msg;
    ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}


void RobotGui::run(){

/*GENERAL INFO */

    // Window Size in Pixel
    cv::Mat frame = cv::Mat(800, 1200, CV_8UC3);

    // Init a OpenCV window and tell cvui to use it.
    cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);

    while (ros::ok()){
    
    // Fill the frame with a nice color
    frame = cv::Scalar(20, 20, 20);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 50, 50, 250, 250, "Topic: " + topic_name_sub);

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, 55, 75, 0.4, 0xff0000, "Data received: %0.2f", data);

/* CMD VEL PUBLISHER */
    
    // Show a button at position x = 100, y = 20
    if (cvui::button(frame, 135, 350, " Forward ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 50
    if (cvui::button(frame, 135, 400, "   Stop  ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 30, y = 50
    if (cvui::button(frame, 50, 400, " Left ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 195, y = 50
    if (cvui::button(frame, 245, 400, " Right ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 80
    if (cvui::button(frame, 135, 450, "Backward")) {
      // The button was clicked,update the Twist message
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, 350, 350, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 375, 375, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);

    // Create window at (320 60) with size 120x40 (width x height) and title
    cvui::window(frame, 350, 400, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 375, 425, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);
    
/* ODOMETRY */
    
    // Square for  X
    cvui::window(frame, 50, 520, 80, 100, "X");
    cvui::printf(frame, 60, 565, 0.9, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.x);
    // Square for  Y
    cvui::window(frame, 150, 520, 80, 100, "Y");
    cvui::printf(frame, 160, 565, 0.9, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.y);
    // Square for  Z
    cvui::window(frame, 250, 520, 80, 100, "Z");
    cvui::printf(frame, 260, 565, 0.9, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.z);
    


/* UPDATE AND SHOW IN SCREEN */

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);


/* STOP THE PROGRAM */

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
    
    
    
    
    }
}
