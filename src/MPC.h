#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  /**
   * Solves the MPC problem given an initial state and waypoint polynomial
   * coefficients. Returns the actuations for the next step, as well as
   * the predicted MPC points beyond that.
   *
   * @param state
   * @param waypoint_coefficients
   * @return vector with following format [<steering-control>, <throttle-control>, x0, y0, x1, y1, x2, y2, ...]
   */
  vector<double> Solve(Eigen::VectorXd& state, Eigen::VectorXd& waypoint_coefficients);
};

#endif /* MPC_H */
