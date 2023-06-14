#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>

using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher cancel_publisher;
MoveBaseClient* ac;

void moveToLocation(double x, double y, double orientation_w) {
  // Create a new goal to send to move_base
  move_base_msgs::MoveBaseGoal goal;

  // Send a goal to the robot
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orientation_w;

  ROS_INFO("Sending goal");
  ac->sendGoal(goal);

  // Wait until the robot reaches the goal
  // ac->waitForResult();

  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has arrived at the goal location");
  else
    ROS_INFO("The robot failed to reach the goal location for some reason");
}

void topic_callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: '%s'", msg->data.c_str());

  std::string command = msg->data;

  if (command == "0") {
    ROS_INFO("STOP");
    actionlib_msgs::GoalID cancel_msg;
    cancel_publisher.publish(cancel_msg);
  } else if (command == "1") {
    // 1번 위치로
    ROS_INFO("Moving to 1st location");
    
    moveToLocation(3.171621, -1.695797, 1.000000);
  } else if (command == "2") {
    // 2번 위치로
    ROS_INFO("Moving to 2nd location");
    moveToLocation(3.694013, -0.148446, 1.000000);
  } else if (command =="3") {
    // 3번 위치로
    ROS_INFO("Moving to 3rd location");
    moveToLocation(-0.527631, 2.464677, 1.000000);
  } else if (command == "4") {
    ROS_INFO("Moving to home");
    moveToLocation(0, 0, 1.0);
  }
}

int main(int argc, char** argv) {
  // Connect to ROS
  ros::init(argc, argv, "app_commander");
  ros::NodeHandle nh;

  MoveBaseClient move_base_client("move_base", true);
  ac = &move_base_client;

  // Wait for the action server to come up so that we can begin processing goals.
  while (!ac->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  cancel_publisher = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

  ros::Subscriber subscriber = nh.subscribe("app_command_data", 10, topic_callback);

  ros::spin();

  return 0;
}