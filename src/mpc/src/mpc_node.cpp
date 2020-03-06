#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "Eigen-3.3.7/Eigen/QR"
#include "mpc.h"

#define POLYFIT_ORDER 2

using namespace std;
using Eigen::VectorXd;

double speed = 1;
vector <geometry_msgs::PointStamped> path_points;

void speedCallback(const std_msgs::Float32::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);
//Convert v, delta and psi to Twist for use with f1 simulator
geometry_msgs::Twist getTwist(double v, double delta, double psi);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher f1sim_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
  ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Publisher polynomial_path = nh.advertise<nav_msgs::Path>("polynomial_path", 1000);
  ros::Subscriber speed_sub = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber closest_path_points = nh.subscribe("closest_path_points", 1000, pathCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;


  Params p;
  int loop_rate;
  double max_vel;

  pnh.param("prediction_horizon", p.prediction_horizon, 10);
  pnh.param("delta_time", p.delta_time, 0.2);
  pnh.param("loop_rate", loop_rate, 10);
  pnh.param("max_mod_delta", p.max_mod_delta, 0.44);
  //pnh.param("max_acceleration", p.max_acceleration, 1.0);
  //pnh.param("max_decceleration", p.max_decceleration, -1.0);
  pnh.param("cte_weight", p.cte_weight, 100);
  pnh.param("epsi_weight", p.epsi_weight, 100);
  pnh.param("delta_weight", p.delta_weight, 2000);
  pnh.param("v_weight", p.v_weight, 100);
  pnh.param("diff_delta_weight", p.diff_delta_weight, 100);
  //pnh.param("diff_a_weight", p.diff_a_weight, 10);
  pnh.param("ref_v", p.ref_v, 4.0);
  pnh.param("max_v", p.max_v, 0.5);
  pnh.param("min_v", p.min_v, -0.1);
  pnh.param("cornering_safety_weight", p.cornering_safety_weight, 1.0);


  MPC mpc(p);
  ros::Rate rate(loop_rate);
  double x, y, orientation;
  Controls controls;

  while(ros::ok())
  {
    if(path_points.empty())
    {
      ros::spinOnce();
      cout << path_points.size() << endl;
      rate.sleep();
      continue;
    }
    //get current state info
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    vector<geometry_msgs::PointStamped> path_points_base_link;
    for(unsigned int i = 0; i < path_points.size(); ++i)
    {
      geometry_msgs::PointStamped pointS;
      listener.transformPoint("/base_link", path_points[i], pointS);
      path_points_base_link.push_back(pointS);
    }

    VectorXd x(path_points.size());
    VectorXd y(path_points.size());
    for(unsigned int i = 0; i < path_points_base_link.size(); ++i)
    {
      x(i) = path_points_base_link[i].point.x;
      y(i) = path_points_base_link[i].point.y;
    }
    VectorXd pathCoeffs;
    pathCoeffs = polyfit(x, y, POLYFIT_ORDER);

    VectorXd state(STATE_VARS);
    state(0) = 0; //x
    state(1) = 0; //y
    tf::Quaternion base_link_rot_qaternion = transform.getRotation();
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 rotation_mat(base_link_rot_qaternion);
    rotation_mat.getRPY(roll, pitch, yaw, 1);
    state(2) = 0; //psi
    state(3) = pathCoeffs[0];
    state(4) = CppAD::atan(pathCoeffs[1]);

    controls = mpc.getControls(pathCoeffs, state);
    //cout << "delta1: " << controls.delta << endl;

    std_msgs::Float64 target_speed_msg;
    std_msgs::Float64 steering_angle_msg;
    nav_msgs::Path optimal_path_msg;
    nav_msgs::Path polynomial_path_msg;

    target_speed_msg.data = controls.velocity;
    //target_speed_msg.data = min(max_vel, max(-1*max_vel, target_speed_msg.data));
    steering_angle_msg.data = controls.delta;
    optimal_path_msg = controls.predicted_path;
    polynomial_path_msg = controls.polynomial_path;

    geometry_msgs::Twist velocity_msg = getTwist(target_speed_msg.data, steering_angle_msg.data, yaw);

    f1sim_cmd.publish(velocity_msg);
    target_speed.publish(target_speed_msg);
    steering_angle.publish(steering_angle_msg);
    optimal_path.publish(optimal_path_msg);
    polynomial_path.publish(polynomial_path_msg);

    ros::spinOnce();
    rate.sleep();
  }
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
    geometry_msgs::PointStamped pointS;
    pointS.point = msg->poses[i].pose.position;
    pointS.header.frame_id = "/map";
    path_points.push_back(pointS);
  }
}


// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  double x_max = xvals[0];
  int valid_points = 1;
  for(int i = 1; i < xvals.size(); ++i)
  {
    if(xvals[i] > xvals[i - 1])
    {
      ++valid_points;
      x_max = xvals[i];
    }
  }


  Eigen::MatrixXd A(valid_points, order + 1);

  for (int i = 0; i < valid_points; ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < valid_points; ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  VectorXd yvals1(valid_points);
  for(int i = 0; i < valid_points; ++i)
  {
    yvals1[i] = yvals[i];
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals1);

  return result;
}

// equations from https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
geometry_msgs::Twist getTwist(double v, double delta, double psi)
{
  cout << "delta: " << delta << endl;
  double beta = atan( (LT - LF)/LT * tan(delta) );
  double ang_vel = v/(LT - LF) * sin(beta);
  double vx = v * cos(psi + beta);
  double vy = v * sin(psi + beta);

  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.linear.z = 0;

  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = ang_vel;

  return cmd_vel;
}
