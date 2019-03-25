#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Constants.h"
#include "FG_eval.h"

using CppAD::AD;
using namespace Constants;

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd& state, Eigen::VectorXd& waypoint_coefficients) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[X_STATE_OFFSET];
  double y = state[Y_STATE_OFFSET];
  double psi = state[PSI_STATE_OFFSET];
  double v = state[VEL_STATE_OFFSET];
  double cte = state[CTE_STATE_OFFSET];
  double epsi = state[EPSI_STATE_OFFSET];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(N_VARIABLES);
  for (int i = 0; i < N_VARIABLES; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(N_VARIABLES);
  Dvector vars_upperbound(N_VARIABLES);
  // TODO: Set lower and upper limits for variables.

  // Set the initial variable values
  vars[X_OFFSET] = x;
  vars[Y_OFFSET] = y;
  vars[PSI_OFFSET] = psi;
  vars[VEL_OFFSET] = v;
  vars[CTE_OFFSET] = cte;
  vars[EPSI_OFFSET] = epsi;

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < STEER_CTRL_OFFSET; i++) {
    vars_lowerbound[i] = THROTTLE_LOWER_BOUND;
    vars_upperbound[i] = THROTTLE_UPPER_BOUND;
  }

  // The upper and lower limits of the steering angle are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = STEER_CTRL_OFFSET; i < THROTTLE_CTRL_OFFSET; i++) {
    vars_lowerbound[i] = STEERING_LOWER_BOUND;
    vars_upperbound[i] = STEERING_UPPER_BOUND;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = THROTTLE_CTRL_OFFSET; i < N_VARIABLES; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(N_CONSTRAINTS);
  Dvector constraints_upperbound(N_CONSTRAINTS);
  for (int i = 0; i < N_CONSTRAINTS; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[X_OFFSET] = x;
  constraints_lowerbound[Y_OFFSET] = y;
  constraints_lowerbound[PSI_OFFSET] = psi;
  constraints_lowerbound[VEL_OFFSET] = v;
  constraints_lowerbound[CTE_OFFSET] = cte;
  constraints_lowerbound[EPSI_OFFSET] = epsi;

  constraints_upperbound[X_OFFSET] = x;
  constraints_upperbound[Y_OFFSET] = y;
  constraints_upperbound[PSI_OFFSET] = psi;
  constraints_upperbound[VEL_OFFSET] = v;
  constraints_upperbound[CTE_OFFSET] = cte;
  constraints_upperbound[EPSI_OFFSET] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(waypoint_coefficients);

  //
  // NOTE: You don't have to worry about these options
  //

  // options for IPOPT solver
  std::string options;

  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";

  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  /*
   * The format of this 'result' vector will be:
   *    [<steering-control>, <throttle-control>, x0, y0, x1, y1, x2, y2, ...]
   */
  vector<double> result;

  result.push_back(solution.x[STEER_CTRL_OFFSET]);
  result.push_back(solution.x[THROTTLE_CTRL_OFFSET]);

  for (int i = 0; i < N-1; i++) {
    result.push_back(solution.x[X_OFFSET + i + 1]);
    result.push_back(solution.x[Y_OFFSET + i + 1]);
  }

  return result;
}
