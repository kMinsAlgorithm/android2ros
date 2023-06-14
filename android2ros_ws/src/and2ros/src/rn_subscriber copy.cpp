#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>

ros::Publisher cancel_publisher;

void topic_callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: '%s'", msg->data.c_str());

  std::string command = msg->data;

  if (command == "0") {
    ROS_INFO("STOP");
    actionlib_msgs::GoalID cancel_msg;
    // cancel_msg에 원하는 값을 설정

    cancel_publisher.publish(cancel_msg);
  }
  else if (command == "1") {
    // 1번 위치로
    ROS_INFO("Moving to 1st location");
    // TODO: 원하는 동작을 추가하세요
  }
  else if (command == "2") {
    // 2번 위치로
    ROS_INFO("Moving to 2nd location");
    // TODO: 원하는 동작을 추가하세요
  }
  else {
    // 3번 위치로
    ROS_INFO("Moving to 3rd location");
    // TODO: 원하는 동작을 추가하세요
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "app_commander");
  ros::NodeHandle nh;

  cancel_publisher = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

  ros::Subscriber subscriber = nh.subscribe("app_command_data", 10, topic_callback);

  ros::spin();

  return 0;
}
