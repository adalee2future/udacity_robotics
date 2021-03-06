#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Pickup Goal
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 2.89;
  goal.target_pose.pose.position.y = 0.09;
  goal.target_pose.pose.orientation.z = 0.95;
  goal.target_pose.pose.orientation.w = 0.32;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCESS: Pickup goal reached!");
  else
    ROS_INFO("FAILURE: Could not reach pickup goal!");

  // Sleep 5 seconds
  ROS_INFO("Sleep for 5 seconds");
  ros::Duration(5.0).sleep();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 3.00;
  goal.target_pose.pose.position.y = 2.68;
  goal.target_pose.pose.orientation.z = -0.45;
  goal.target_pose.pose.orientation.w = 0.89;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCESS: Drop off goal reached!");
  else
    ROS_INFO("FAILURE: Could not reach drop off goal!");

  return 0;
}

