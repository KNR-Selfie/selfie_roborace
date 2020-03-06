#include <cppad_spline.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

std::vector<geometry_msgs::PointStamped> path_points;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  path_points.clear();
  for(unsigned int i = 0; i < msg->poses.size(); ++i)
  {
    geometry_msgs::PointStamped p;
    p.point = msg->poses[i].pose.position;
    p.header.frame_id = msg->header.frame_id;
    path_points.push_back(p);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cppad_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::Path>("polynomial_path", 1);
  ros::Subscriber sub = nh.subscribe("closest_path_points", 1, pathCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Rate rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    if (path_points.empty()) continue;

    listener.lookupTransform("/skidpad", "/base_link", ros::Time(0), transform);

    std::vector<double> pts_x, pts_y;
    for (unsigned int i = 0; i < path_points.size(); ++i)
    {
      geometry_msgs::PointStamped p;
      listener.transformPoint("/base_link", path_points[i], p);
      pts_x.push_back(p.point.x);
      pts_y.push_back(p.point.y);

      ROS_INFO("%f, %f", p.point.x, p.point.y);
    }

    CppAD::spline ref_spline(pts_x, pts_y);

    // Publish interpolated path

    nav_msgs::Path interpolated_path;
    interpolated_path.header.frame_id = "base_link";
    for (double x = -0.5; x <= 2.0; x += 0.1)
    {
      double y = CppAD::Value(ref_spline(x));

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = y;

      interpolated_path.poses.push_back(pose);
    }

    pub.publish(interpolated_path);
  }
}
