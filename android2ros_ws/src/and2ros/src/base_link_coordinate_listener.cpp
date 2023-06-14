#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_link_coordinate_listener");
  ros::NodeHandle nh;

  // 좌표 변환에 사용할 객체 생성
  tf::TransformListener listener;

  ros::Rate rate(10.0);  // 10Hz로 루프 실행

  while (ros::ok()) {
    // 현재 시간을 기준으로 최신의 좌표 변환을 받기 위해 시간 설정
    ros::Time now = ros::Time::now();

    // 좌표 변환 결과를 저장할 변수
    tf::StampedTransform transform;

    try {
      // "base_link" 좌표 프레임으로부터 "map" 좌표 프레임으로의 변환 수행
      listener.waitForTransform("map", "base_link", now, ros::Duration(1.0));
      listener.lookupTransform("map", "base_link", now, transform);

      // 변환된 좌표 출력
      ROS_INFO("Base Link Coordinates (Map Frame): (%f, %f, %f)",
               transform.getOrigin().x(),
               transform.getOrigin().y(),
               transform.getOrigin().z());
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Failed to get base link coordinates: %s", ex.what());
    }

    rate.sleep();
  }

  return 0;
}
