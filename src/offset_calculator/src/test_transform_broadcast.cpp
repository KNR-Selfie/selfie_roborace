#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#define UPDATE_RATE 1

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_transform_broadcast");
  ros::NodeHandle node;
  ros::Rate rate(UPDATE_RATE);

  tf::TransformBroadcaster tf_br;
  tf::Transform transform;

  double x = 0, y = 0;

  while(node.ok())
  {
    transform.setOrigin( tf::Vector3(x + 3.0, y + 2.0, 0.0));
    transform.setRotation( tf::Quaternion(0, 0, 0.5735764, 0.819152 ));
    tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    rate.sleep();
  }

  return 0;
}
