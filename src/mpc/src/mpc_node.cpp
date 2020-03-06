#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <iterator>
#include <algorithm>
#include <fstream>
#include "mpc.h"

using namespace std;

double speed = 0;
Params p;
bool ready = false;
std::vector<geometry_msgs::PointStamped> path_points;
bool updatePoints = false;


void speedCallback(const std_msgs::Float32::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Publisher polynomial_path = nh.advertise<nav_msgs::Path>("polynomial_path", 1000);
  ros::Publisher drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1000);
  // order of the forces - [Ffx, Ffy, Frx, Fry, |Ff|, |Fr|]
  ros::Subscriber speed_sub = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber path_sub = nh.subscribe("closest_path_points", 1000, pathCallback);
  tf::StampedTransform transform;
  tf::TransformListener listener;

  double max_steering_angle;
  double v_max, v_min;
  int loop_rate;

  pnh.param("loop_rate", loop_rate, 50);
  pnh.param("prediction_horizon", p.prediction_horizon, 10);
  pnh.param("dt", p.dt, 0.2);
  pnh.param("max_steering_angle", max_steering_angle, 0.44);
  pnh.param("w_cte", p.w_cte, 100.0);
  pnh.param("w_eps", p.w_eps, 100.0);
  pnh.param("w_delta_var", p.w_delta_var, 2000.0);
  pnh.param("w_delta", p.w_delta, 20.0);
  pnh.param("w_v", p.w_v, 100.0);
  pnh.param("w_a", p.w_a, 200.0);
  pnh.param("w_a_var", p.w_a_var, 100.0);
  pnh.param("v_ref", p.v_ref, 0.5);
  pnh.param("v_max", v_max, 0.5);
  pnh.param("v_min", v_min, -0.1);
  pnh.param("a_max", p.a_max, 0.8);
  pnh.param("an_max", p.an_max, 1.0);
  pnh.param("ipopt_cpu_time", p.ipopt_cpu_time, 0.2);
  pnh.param("spline_visualization_delta", p.spline_visualization_delta, 0.1);
  pnh.param("spline_visualization_points", p.spline_visualization_points, 20);
  pnh.param("lf", p.lf, 0.25);
  pnh.param("lr", p.lr, 0.25);
  // x, y, psi, v
  p.state_vars = 4;
  // a, delta
  p.steering_vars = 2;
  p.constraint_functions = 5;
  p.newPoints = false;

  double avg_acceleration = 0;
  double loop_count = 1;
  ros::Rate rate(loop_rate);
  ros::spinOnce();
  ros::Duration(2.0).sleep();
  double last_delta = 0, last_acceleration = 0;

  std::ofstream data_file;
  data_file.open("/home/marcel/ros_ws/analytics_ws/src/performance_monitor/data/acceleration_data.csv");
  data_file << "rostime,an,an_nl,at\n";

  while(ros::ok())
  {
    //get current state info

    if(!ready) {
      std::cout << "Revving up\n";
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    std::vector<double> state(p.state_vars);
    state[0] = 0; //x
    state[1] = 0; //y
    state[2] = 0; //psi
    state[3] = speed;

    MPC mpc;
    Controls controls;

    std::vector<double> state_lower{-1e9, -1e9, -1e9, v_min};

    std::vector<double> state_upper{1e9, 1e9, 1e9, v_max};

    std::vector<double> steering_lower{-max_steering_angle, -p.a_max};

    std::vector<double> steering_upper{max_steering_angle, p.a_max};

    if(updatePoints)
    {
      double prev = -10000000;
      p.pts_x.clear();
      p.pts_y.clear();
      for (unsigned int i = 0; i < path_points.size(); ++i)
      {
        geometry_msgs::PointStamped point;
        listener.transformPoint("/base_link", path_points[i], point);
        if(point.point.x <= prev)
          continue;
        prev = point.point.x;
        p.pts_x.push_back(point.point.x);
        p.pts_y.push_back(point.point.y);
      }
      p.newPoints = true;
    }
    cout << endl;
    clock_t time = clock();

    controls = mpc.mpc_solve(state, state_lower, state_upper, steering_lower,
                             steering_upper, p, last_acceleration, last_delta);
    time = clock() - time;
    std::cout << "mpc_exec_time: " << (double)time/CLOCKS_PER_SEC << std::endl;

    nav_msgs::Path optimal_path_msg;
    nav_msgs::Path polynomial_path_msg;

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp.sec = ros::Time::now().sec;
    drive_msg.header.stamp.nsec = ros::Time::now().nsec;
    drive_msg.drive.steering_angle = controls.delta;
    drive_msg.drive.speed = controls.velocity;
    drive_msg.drive.acceleration = controls.acceleration;

    last_delta = controls.delta;
    last_acceleration = controls.acceleration;

    data_file << ros::Time::now() << "," <<
    controls.get_normal_acceleration_linear(p.lr, p.lf) << "," <<
    controls.get_normal_acceleration_non_linear(p.lr, p.lf) << "," <<
    controls.get_tangential_acceleration() << "\n";
    ++loop_count;

    optimal_path_msg = controls.predicted_path;
    polynomial_path_msg = controls.polynomial_path;

    drive.publish(drive_msg);
    optimal_path.publish(optimal_path_msg);
    polynomial_path.publish(polynomial_path_msg);

    ros::spinOnce();

    rate.sleep();
  }

  data_file.close();
  return 0;
}


void speedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  speed = msg->data;
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{

  path_points.clear();
  for(unsigned int i = 0; i < msg->poses.size(); ++i)
  {
    geometry_msgs::PointStamped point;
    point.point = msg->poses[i].pose.position;
    point.header.frame_id = msg->header.frame_id;
    path_points.push_back(point);
  }
  updatePoints = true;
  ready = true;

}
