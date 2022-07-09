#include <string>
#include <iostream>
#include <typeinfo>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Drive robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service move");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    ROS_INFO("process_image_callback func");
    int white_pixel = 255;

    double direction = 0.0;
    bool has_ball = false;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for (int i = 0; i < img.height * img.width; i = i + 3) {
        int val_r = img.data[i * 3];
        int val_g = img.data[i * 3 + 1];
        int val_b = img.data[i * 3 + 2];

        if (val_r == white_pixel and val_g == white_pixel and val_b == white_pixel) {
	    cout << "white pixel found" << endl;
	    has_ball = true;
	    int step0 = i % img.width;
	    if (step0 < 1.0 * 14 / 45 * img.width) {direction = 1.0;}
	    if (step0 > 1.0 * 31 / 45 * img.width) {direction = -1.0;}
	    cout << "step0 and width" << step0 << " " << img.width << endl;
	    cout << "direction " << direction << endl;
	        
	    break;
	}
    }
    double vel;
    double ang;
    if (has_ball) {
      vel = 0.5 / 2;
      ang = direction * 0.3 / 2;
    }
    else {
     vel = 0;
     ang = 0;
    }
    cout << "vel and ang: " << vel << " " << ang << endl;
    drive_robot(vel, ang);
}

int main(int argc, char** argv)
{
    ROS_INFO("main func in process_image.cpp");
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO("ready to chase a ball");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
