//
// Created by Admin on 2019-03-24.
//

#include "FG_eval.h"

using namespace Constants;

void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // Cost related to the reference state (open-ended):
    for (int i = 0; i < N; i++) {
        fg[0] += 3000*CppAD::pow(vars[CTE_OFFSET + i], 2);
        fg[0] += 3000*CppAD::pow(vars[EPSI_OFFSET + i], 2);
        fg[0] += CppAD::pow(vars[VEL_OFFSET + i] - TARGET_VELOCITY, 2);
    }

    for (int i = 0; i < N - 1; i++) {
        fg[0] += 5*CppAD::pow(vars[STEER_CTRL_OFFSET + i], 2);                        // Stabilize steering
        fg[0] += 5*CppAD::pow(vars[THROTTLE_CTRL_OFFSET + i], 2);                     // Stabilize throttle
        fg[0] += 700*CppAD::pow(vars[STEER_CTRL_OFFSET + i] * vars[VEL_OFFSET+i], 2); // Penalty for speed + steer
    }

    for (int i = 0; i < N - 2; i++) {
        fg[0] += 300*CppAD::pow(vars[STEER_CTRL_OFFSET + i + 1] - vars[STEER_CTRL_OFFSET + i], 2); // 200
        fg[0] += 20*CppAD::pow(vars[THROTTLE_CTRL_OFFSET + i + 1] - vars[THROTTLE_CTRL_OFFSET + i], 2); //10
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + X_OFFSET] = vars[X_OFFSET];
    fg[1 + Y_OFFSET] = vars[Y_OFFSET];
    fg[1 + PSI_OFFSET] = vars[PSI_OFFSET];
    fg[1 + VEL_OFFSET] = vars[VEL_OFFSET];
    fg[1 + CTE_OFFSET] = vars[CTE_OFFSET];
    fg[1 + EPSI_OFFSET] = vars[EPSI_OFFSET];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
        AD<double> x1 = vars[X_OFFSET + t];
        AD<double> x0 = vars[X_OFFSET + t - 1];
        AD<double> y1 = vars[Y_OFFSET + t];
        AD<double> y0 = vars[Y_OFFSET + t - 1];
        AD<double> psi1 = vars[PSI_OFFSET + t];
        AD<double> psi0 = vars[PSI_OFFSET + t - 1];
        AD<double> v1 = vars[VEL_OFFSET + t];
        AD<double> v0 = vars[VEL_OFFSET + t - 1];
        AD<double> cte1 = vars[CTE_OFFSET + t];
        AD<double> cte0 = vars[CTE_OFFSET + t - 1];
        AD<double> epsi1 = vars[EPSI_OFFSET + t];
        AD<double> epsi0 = vars[EPSI_OFFSET + t - 1];
        AD<double> a = vars[THROTTLE_CTRL_OFFSET + t - 1];
        AD<double> delta = vars[STEER_CTRL_OFFSET + t - 1];
        if (t > 1) {   // use previous actuations (to account for latency)
            a = vars[THROTTLE_CTRL_OFFSET + t - 2];
            delta = vars[STEER_CTRL_OFFSET + t - 2];
        }
        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // NOTE: The use of `AD<double>` and use of `CppAD`!
        // This is also CppAD can compute derivatives and pass
        // these to the solver.

        // TODO: Setup the rest of the model constraints
        fg[1 + X_OFFSET + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + Y_OFFSET + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + PSI_OFFSET + t] = psi1 - (psi0 - v0/L_F * delta * dt);
        fg[1 + VEL_OFFSET + t] = v1 - (v0 + a * dt);
        fg[1 + CTE_OFFSET + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + EPSI_OFFSET + t] = epsi1 - ((psi0 - psides0) - v0/L_F * delta * dt);
    }
}
