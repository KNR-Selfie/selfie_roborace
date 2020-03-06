#ifndef MPC_H
#define MPC_H
#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3.7/Eigen/Core"
#include <vector>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#define STATE_VARS 5
#define ACTUATORS_VARS 2
// length from rear axis to COG
#define LF 0.19
// total length
#define LT 0.34

using CppAD::AD;
using Eigen::VectorXd;

//Struct that is used to initialize the MPC
struct Params
{
  int prediction_horizon;
  int cte_weight;
  int epsi_weight;
  int v_weight;
  int delta_weight;
  int a_weight;
  int diff_delta_weight;
  int diff_v_weight;
  double delta_time;
  double max_mod_delta;
  //double max_acceleration;
  //double max_decceleration;
  double ref_v;
  double max_v;
  double min_v;
  double cornering_safety_weight;
};

// Struct that is returned by MPC
struct Controls
{
  double velocity;
  double delta;
  nav_msgs::Path predicted_path;
  nav_msgs::Path polynomial_path;
};

//Interface for interactions with ros
class MPC
{

public:
  // Constructor
  MPC(Params p);
  // Main method of the MPC
  Controls getControls(Eigen::VectorXd pathCoeffs, const VectorXd &state);

};

// Class that calculates the cost function f and contraints g_i
class FG_eval
{

public:
    //Coefficients of a polynomial fitted to closest path waypoints in the frame of the car
    Eigen::VectorXd pathCoeffs;
    FG_eval(Eigen::VectorXd pathCoeffs);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars);
};

//Function that optimizes the cost function
std::vector<double> Solve(const VectorXd &state, const VectorXd &coeffs);

#endif //MPC_H
