#ifndef MPC_H
#define MPC_H
#include <cmath>

struct Params{
	double dt;
	double lr;
	double lf;
	double v_ref;
	double a_max;
    double an_max;

	std::vector<double> pts_x, pts_y;
	bool newPoints;

	double w_a;
	double w_cte;
	double w_eps;
	double w_v;
	double w_delta_var;
	double w_delta;
	double w_a_var;
	double sigmoid_k;
	double ipopt_cpu_time;
	size_t state_vars;
	size_t steering_vars;
	int prediction_horizon;
	int spline_visualization_points;
	double spline_visualization_delta;
	size_t constraint_functions;
	// order of state variables
	size_t x = 0, y = 1, psi = 2, v = 3;
	// order of steering variables
	size_t delta = 0, a = 1;
};

struct Controls{
  double delta;
  double velocity;
  double acceleration;
  nav_msgs::Path predicted_path;
  nav_msgs::Path polynomial_path;

  double get_total_acceleration2(double lr, double lf){
      double at2 = acceleration*acceleration;
      double beta = atan(lr/(lr + lf) * tan(delta));
      double an = velocity*velocity * sin(beta)/lr;
      double an2 = an*an;
      return at2 + an2;
  }

  double get_normal_acceleration_linear(double lr, double lf){
      double beta = (lr/(lr + lf)) * delta;
	  return velocity*velocity * beta/lr;
  }

  double get_normal_acceleration_non_linear(double lr, double lf){
	  double beta = atan(lr/(lr + lf) * tan(delta));
      return velocity*velocity * sin(beta)/lr;
  }

  double get_tangential_acceleration(){
      return acceleration;
  }



};


//Interface for interactions with ros
class MPC{

public:

  MPC(){

  }
  // Main method of the MPC
  Controls mpc_solve(std::vector<double> state0, std::vector<double> state_lower,
  				 std::vector<double> state_upper, std::vector<double> steering_lower,
           std::vector<double> steering_upper, Params p, double last_acceleration,
	       double last_delta);

};


#endif //MPC_H
