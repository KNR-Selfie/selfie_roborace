#include "mpc.h"
#include <cassert>

//Parameters
Params params;
// Position on the map
size_t x_start;
size_t y_start;
// Orientation with respect to the x(?) axis
size_t psi_start;
// Velocity with respect to map
size_t v_start;
// Path position offset
size_t cte_start;
// Path heading offset
size_t epsi_start;
// Variables for actuators
// Steering angle
size_t delta_start;
// Acceleration
//size_t a_start;


MPC::MPC(Params p)
{
  params = p;
  size_t N = params.prediction_horizon;
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  cte_start = psi_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  v_start = delta_start + N - 1;
}


Controls MPC::getControls(Eigen::VectorXd pathCoeffs, const VectorXd &state)
{
  //Get the optimal controls
  FG_eval fg_eval(pathCoeffs);
  std::vector<double> solution;
  solution = Solve(state, pathCoeffs);

  //Return the controls
  Controls ret;
  ret.delta = solution[0];
  ret.velocity = solution[1];

  size_t N = params.prediction_horizon;
  assert(N > 0);
  //Return predicted path
  std::vector <geometry_msgs::PoseStamped> poses(N);
  std::vector <geometry_msgs::PoseStamped> polynomial_poses(N);

  for (int i = 0; i < N; i++)
  {
    poses[i].header.stamp = ros::Time::now();
    poses[i].header.frame_id = "base_link";
    poses[i].pose.position.x = solution[2 * i + 2];
    poses[i].pose.position.y = solution[2 * i + 3];
  }

  double x = 0.1;

  for (int i = 0; i < N; i++)
  {
    polynomial_poses[i].header.stamp = ros::Time::now();
    polynomial_poses[i].header.frame_id = "base_link";
    x += x;
    polynomial_poses[i].pose.position.x = x;
    polynomial_poses[i].pose.position.y = pathCoeffs[0] + pathCoeffs[1] * x + pathCoeffs[2]* x * x;
  }

  std::swap(ret.polynomial_path.poses, polynomial_poses);
  ret.polynomial_path.header.frame_id = "base_link";
  ret.polynomial_path.header.stamp = ros::Time::now();

  std::swap(ret.predicted_path.poses, poses);
  ret.predicted_path.header.frame_id = "base_link";
  ret.predicted_path.header.stamp = ros::Time::now();

  return ret;

}


std::vector<double> Solve(const VectorXd &state, const VectorXd &pathCoeffs)
{
  std::cout << "Im loop" << std::endl;
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  /*
    Initiate variables describing a state of the car
  */
  //Position of the car on the map
  double x = state[0];
  double y = state[1];
  //Angle with respect to the x(?) axis
  double psi = state[2];
  //Distance from the desired path
  double cte = state[3];
  //Heading offset from the desired path
  double epsi = state[4];

  /*
   * Initiate indexing variables //TODO ogarnac zeby to bylo dzielone z fg_eval
   */
  size_t N = params.prediction_horizon;
  //Set the number of model variables
  size_t n_vars = STATE_VARS * N + ACTUATORS_VARS * (N - 1);
  //Set the number of constraints
  size_t n_constraints = STATE_VARS * N;

  //Initial value of the independent variables
  //SHOULD BE 0 besides initial state
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i)
  {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  /*
   * Set lower and upper limits for state variables
   */
  for (size_t i = 0; i < delta_start; ++i)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  /*
   * Set initial state variables
   */
  vars_lowerbound[x_start] = x;
  vars_upperbound[x_start] = x;
  vars_lowerbound[y_start] = y;
  vars_upperbound[y_start] = y;
  vars_lowerbound[psi_start] = psi;
  vars_upperbound[psi_start] = psi;
  vars_lowerbound[cte_start] = cte;
  vars_upperbound[cte_start] = cte;
  vars_lowerbound[epsi_start] = epsi;
  vars_upperbound[epsi_start] = epsi;

  // The upper and lower limits of delta are set to -25 and 25
  // degrees in radians
  for (size_t i = delta_start; i < v_start; ++i)
  {
    vars_lowerbound[i] = -1 * params.max_mod_delta;
    vars_upperbound[i] = params.max_mod_delta;
  }

  //Acceleration/decceleration upper and lower limits
  for (size_t i = v_start; i < n_vars; ++i)
  {
    vars_lowerbound[i] = params.min_v;
    vars_upperbound[i] = params.max_v;
  }

  // Lower and upper limits for the constraints.
  // The constraints are defined as difference between a state variable provided by
  // the ipopt optimizer and a state variable resulting from the model
  // equations, so they should be always equal to 0, because we only want to do
  // calculations for states that can actually happen considering the initial
  // state and bounds on actuators
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Object that computes the cost function f and the constraints g_i
  FG_eval fg_eval(pathCoeffs);

  //options for IPOPT optimizer
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // Object that is used to store the solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Optimize the cost function
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // Display the cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return only the first actuator values
  std::vector<double> ret_val;
  ret_val.push_back(solution.x[delta_start]);
  ret_val.push_back(solution.x[v_start]);

  for(size_t i = 0; i < N - 1; ++i) std::cout << "delta: " << solution.x[delta_start + i] << " v: " << solution.x[v_start + i] << std::endl;

  // Also return the optimal positions to display predicted path in rviz
  for (size_t i = 0; i < N; ++i)
  {
    ret_val.push_back(solution.x[x_start + i]);
    ret_val.push_back(solution.x[y_start + i]);
  }
  std::cout << "Loop end" << std::endl;
  return ret_val;
}

FG_eval::FG_eval (VectorXd pathCoeffs)
{
    this->pathCoeffs = pathCoeffs;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    fg[0] = 0;
    size_t N = params.prediction_horizon;
    // The part of the cost based on the reference state
    for (unsigned int t = 0; t < N; ++t)
    {
        fg[0] += params.cte_weight * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += params.epsi_weight * CppAD::pow(CppAD::sin(vars[epsi_start + t]), 2);
    }

    // Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; ++t)
    {
        fg[0] += params.delta_weight * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += params.v_weight * CppAD::pow(params.max_v - vars[v_start + t] , 2);
    }

    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; ++t)
    {
        fg[0] += params.diff_delta_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += params.diff_v_weight * CppAD::pow((vars[v_start + t + 1] - vars[v_start + t])/params.delta_time, 2);
    }

    // Make cornering safer
    for (unsigned int t = 0; t < N - 1; ++t)
    {
        fg[0] += params.cornering_safety_weight * CppAD::pow(vars[v_start + t] * vars[v_start + t] * vars[delta_start + t], 2);
    }

    //Optimizer constraints - g(x)
    fg[1 + x_start] = 0;
    fg[1 + y_start] = 0;
    fg[1 + psi_start] = 0;
    fg[1 + cte_start] = 0;
    fg[1 + epsi_start] = 0;

    for (unsigned int t = 1; t < N; ++t)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // The state at time t.
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];

        AD<double> f1 = pathCoeffs[0] + pathCoeffs[1] * x1 + pathCoeffs[2] * x1*x1;
        AD<double> psides1 = CppAD::atan(pathCoeffs[1] + 2 * pathCoeffs[2] * x1);

        double dt = params.delta_time;
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 + v0 / LT * delta0 * dt);
        fg[1 + cte_start + t] = cte1 - (f1 - y1);
        fg[1 + epsi_start + t] = epsi1 - (psides1 - psi1);
    }
}
