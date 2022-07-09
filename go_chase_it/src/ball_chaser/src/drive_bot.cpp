#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

//TODO: Include the ball_chaser "DriveToTarget" header file
#include <ball_chaser/DriveToTarget.h>

//i ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;


// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received");
    geometry_msgs::Twist geo_msg;
    geo_msg.linear.x = req.linear_x;
    geo_msg.angular.z = req.angular_z;
    motor_command_publisher.publish(geo_msg);

    ros::Duration(0).sleep();
    res.msg_feedback = "Target set " + std::to_string(geo_msg.linear.x) + " " + std::to_string(geo_msg.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward 
    motor_command.linear.x = geo_msg.linear.x;
    motor_command.angular.z = geo_msg.angular.z;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("ready to drive robots");


    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
