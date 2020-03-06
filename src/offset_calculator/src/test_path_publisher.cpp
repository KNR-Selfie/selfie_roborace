#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#define UPDATE_RATE 10
#define QUEUE_SIZE 10

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_path_publisher");
  ros::NodeHandle node;
  ros::Rate rate(UPDATE_RATE);

  nav_msgs::Path path;

  std::vector <geometry_msgs::PoseStamped> poses(5);

  for(int i = 0; i < 5; i ++)
  {
    poses[i].header.stamp = ros::Time::now();
    poses[i].header.frame_id = "map";
    poses[i].pose.position.x = 2 + i*0.1;
    poses[i].pose.position.y = 1 + i*0.15;
    poses[i].pose.position.z = 0;
  }

  std::swap(path.poses, poses);
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  ros::Publisher path_pub;
  path_pub = node.advertise<nav_msgs::Path>("test_path", QUEUE_SIZE);

  while(node.ok())
  {
    path_pub.publish(path);
  }

}
