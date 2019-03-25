//
// Created by safdar on 2019-03-24.
//

#ifndef MPC_FG_EVAL_H
#define MPC_FG_EVAL_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Constants.h"

using CppAD::AD;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(const Eigen::VectorXd& coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars);
};

#endif //MPC_FG_EVAL_H
