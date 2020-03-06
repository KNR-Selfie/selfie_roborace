#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#undef HAVE_STDDEF_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <cppad_spline.hpp>
#include "mpc.h"

using CppAD::AD;
using namespace std;

CppAD::spline* spline = nullptr;

class FG_eval {

	Params p;

public:
	typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;

	FG_eval(Params p){
		this->p = p;
	}

	void operator()(ADvector& fg, const ADvector& xi){

		size_t steering_start = p.state_vars * (p.prediction_horizon + 1);

		for(int t = 0; t < p.prediction_horizon; ++t){

			// index of steering variables
			size_t iu0 = steering_start + p.steering_vars * t;
			size_t iu1 = steering_start + p.steering_vars * (t + 1);
			// index of state variables
			size_t is0 = p.state_vars * t;
			size_t is1 = p.state_vars * (t + 1);


			AD<double> x0 = xi[p.x + is0];
			AD<double> y0 = xi[p.y + is0];
			AD<double> psi0 = xi[p.psi + is0];
			AD<double> v0 = xi[p.v + is0];
			AD<double> a0 = xi[p.a + iu0];
			AD<double> delta0 = xi[p.delta + iu0];

			AD<double> x1 = xi[p.x + is1];
			AD<double> y1 = xi[p.y + is1];
			AD<double> psi1 = xi[p.psi + is1];
			AD<double> v1 = xi[p.v + is1];
			AD<double> a1 = xi[p.a + iu1];
			AD<double> delta1 = xi[p.delta + iu1];

			AD<double> v_avg = v0 + 0.5*a0*p.dt;

			AD<double> beta0 = p.lr/(p.lf + p.lr) * delta0;
			AD<double> beta1 = p.lr/(p.lf + p.lr) * delta1;
			AD<double> at = a0;
			AD<double> an = v0*v0*beta0/p.lr;
			AD<double> a_max = p.a_max;
			// course trajectory error
			AD<double> y_path = (*spline)(x0);

			AD<double> psi_path = CppAD::atan(spline->deriv(x0));

			//AD<double> a_max2 = CppAD::pow(a_max, 2);
            AD<double> k = p.sigmoid_k;
            AD<double> w_a = p.w_a;
			// course trajectory error
			fg[0] += p.w_cte * CppAD::pow(y0 - y_path, 2);
			// course heading error
			fg[0] += p.w_eps * CppAD::pow(psi0 - psi_path, 2);
			// velocity error
			fg[0] += p.w_v * CppAD::pow(v_avg - p.v_ref, 2);
			// penalize bigger steering angles
			fg[0] += p.w_delta * CppAD::pow(delta0, 2);
			// sequential actuations
			fg[0] += p.w_delta_var * CppAD::pow(delta1 - delta0, 2);
			fg[0] += p.w_a_var * CppAD::pow(a1 - a0, 2);

            // constraints
			fg[1 + p.constraint_functions * t] = x1 - (x0 + v_avg * p.dt * CppAD::cos(psi0 + beta0));
			fg[2 + p.constraint_functions * t] = y1 - (y0 + v_avg * p.dt * CppAD::sin(psi0 + beta0));
			fg[3 + p.constraint_functions * t] = psi1 - (psi0 + v_avg * p.dt * beta0/p.lr);
			fg[4 + p.constraint_functions * t] = v1 - (v0 + a0 * p.dt);
      		fg[5 + p.constraint_functions * t] = CppAD::pow(an, 2) + CppAD::pow(at, 2);

		}
		return;
	}
};


Controls MPC::mpc_solve(std::vector<double> state0, std::vector<double> state_lower,
				 std::vector<double> state_upper, std::vector<double> steering_lower,
				 std::vector<double> steering_upper, Params p, double last_acceleration,
			     double last_delta){
	typedef CPPAD_TESTVECTOR( double ) Dvector;

	if(p.newPoints)
	{
      if(spline != nullptr)
			    delete spline;
			spline = new CppAD::spline(p.pts_x, p.pts_y);
	}

	// number of independent state and steering variables (domain dimension for f and g)
	size_t number_of_variables = p.state_vars * (p.prediction_horizon + 1)
	                           + p.steering_vars * (p.prediction_horizon + 1);
	// number of constraints (range dimension for g)
	size_t number_of_constraints = p.constraint_functions * p.prediction_horizon;
  size_t steering_start = p.state_vars*(p.prediction_horizon + 1);
	// initial value of the independent variables
	Dvector xi(number_of_variables); for(int i = 0; i < xi.size(); ++i) xi[i] = 0;

	// lower and upper limits for independent variables
	Dvector xi_lower(number_of_variables), xi_upper(number_of_variables);


	// lower and upper limits for state variables
  for(int i = 0; i < p.state_vars; ++i){
    xi_lower[i] = state0[i]; xi_upper[i] = state0[i];
  }

	for(int i = 0; i < p.state_vars; ++i){
		for(int j = i + p.state_vars; j < steering_start; j += p.state_vars){
			xi_lower[j] = state_lower[i]; xi_upper[j] = state_upper[i];
		}
	}


	// lower and upper limits for steering variables

	for(int i = 0; i < p.steering_vars; ++i){
		for(int j = i + steering_start; j < xi.size(); j += p.steering_vars){
			xi_lower[j] = steering_lower[i]; xi_upper[j] = steering_upper[i];
		}
	}

	// lower and upper limits for g
	Dvector g_lower(number_of_constraints), g_upper(number_of_constraints);
    g_lower[0] = 0; g_upper[0] = 0;

    for(int i = 0; i < p.prediction_horizon; i++){

        g_lower[i * p.constraint_functions] = 0;
        g_upper[i * p.constraint_functions] = 0;

        g_lower[1 + i * p.constraint_functions] = 0;
        g_upper[1 + i * p.constraint_functions] = 0;

        g_lower[2 + i * p.constraint_functions] = 0;
        g_upper[2 + i * p.constraint_functions] = 0;

        g_lower[3 + i * p.constraint_functions] = 0;
        g_upper[3 + i * p.constraint_functions] = 0;

        g_lower[4 + i * p.constraint_functions] = 0;
        g_upper[4 + i * p.constraint_functions] = p.a_max * p.a_max;
    }
	// object that computes objective and constraints
	FG_eval fg_eval(p);

	// options
	std::string options;
	// turn off any printing
  options += "Integer print_level  2\n";
  //options += "Integer acceptable_iter         100\n";
  //options += "Numeric acceptable_constr_viol_tol         0.01\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  string ipopt_cpu_time_string = to_string(p.ipopt_cpu_time);
  options += "Numeric max_cpu_time          " + ipopt_cpu_time_string + "\n";
	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, xi, xi_lower, xi_upper, g_lower, g_upper, fg_eval, solution
	);

    std::cout << "cost: " << solution.obj_value << "\n";

	// ============== DEBUG =====================
	/*std::cout << "cost: " << solution.obj_value << "\n";

	for(int i = 0; i < steering_start; ++i){
    if(i % p.state_vars == 0) std::cout << "\n";
		std::cout << solution.x[i] << " ";
	}
  std::cout << "\n";
  for(int i = steering_start; i < solution.x.size(); ++i){
    if(i % p.steering_vars == 0) std::cout << "\n";
		std::cout << solution.x[i] << " ";
	}
	std::cout << "\n";
	double delta1 = solution.x[steering_start + p.steering_vars];
	double beta1 = atan(p.lr/(p.lf + p.lr) * tan(delta1));
	double at = solution.x[steering_start + p.steering_vars + 1];
	double v1 = solution.x[p.state_vars + p.v];
	double an = v1*v1*sin(beta1)/p.lr;
	double a_tot = sqrt(an*an + at*at);
	std::cout << "total acceleration: " << sqrt(an*an + at*at) << "\n";

	std::ofstream file;
	file.open("~/cost-acceleration.csv", std::fstream::out | std::fstream::app);
	file << a_tot << "," << solution.obj_value << "\n";*/
	// ============== DEBUG =====================
  Controls controls;

  controls.delta = solution.x[steering_start + p.delta];
  controls.velocity = solution.x[p.state_vars + p.v];
  controls.acceleration = solution.x[steering_start + p.a];

  std::vector <geometry_msgs::PoseStamped> poses(p.prediction_horizon);
  std::vector <geometry_msgs::PoseStamped> polynomial_poses(p.spline_visualization_points);

  for(int i = p.state_vars; i < steering_start; i += p.state_vars){
    int j = i/p.state_vars - 1;
    poses[j].header.stamp = ros::Time::now();
    poses[j].header.frame_id = "base_link";
    poses[j].pose.position.x = solution.x[i + p.x];
    poses[j].pose.position.y = solution.x[i + p.y];
  }

  for (int i = 0; i < p.spline_visualization_points; i++)
  {
    polynomial_poses[i].header.stamp = ros::Time::now();
    polynomial_poses[i].header.frame_id = "base_link";
    polynomial_poses[i].pose.position.x = i*p.spline_visualization_delta;
    polynomial_poses[i].pose.position.y = CppAD::Value((*spline)(i*p.spline_visualization_delta));
  }
  // DEBUG
  // cout << "splojn - predicted_y\n";
  // for(int i = 0; i < poses.size(); ++i){
	//   cout << (*spline)(poses[i].pose.position.x) - poses[i].pose.position.y << ", ";
  // }
  // cout << "\n";

  std::swap(controls.polynomial_path.poses, polynomial_poses);
  controls.polynomial_path.header.frame_id = "base_link";
  controls.polynomial_path.header.stamp = ros::Time::now();

  std::swap(controls.predicted_path.poses, poses);
  controls.predicted_path.header.frame_id = "base_link";
  controls.predicted_path.header.stamp = ros::Time::now();



  return controls;

}
