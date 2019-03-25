//
// Created by Admin on 2019-03-24.
//

#ifndef MPC_CONSTANTS_H
#define MPC_CONSTANTS_H

namespace Constants {
    /**
     * MPC hyper parameters:
     */
    const size_t N = 10;          // Prediction horizon
    const double dt = 0.1;        // Sample size (seconds)

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    const size_t N_VARIABLES = N * 6 + (N - 1) * 2;
    // TODO: Set the number of constraints
    const size_t N_CONSTRAINTS = N * 6;

    /*
     * Hard constraints
     */
    const double THROTTLE_LOWER_BOUND = -1.0e19;
    const double THROTTLE_UPPER_BOUND = 1.0e19;
    const double STEERING_LOWER_BOUND = -0.436332;
    const double STEERING_UPPER_BOUND = 0.436332;

    /**
     * Order of the polynomial used to fit the waypoints
     */
    const size_t POLYNOMIAL_ORDER = 3;

    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    const double L_F = 2.67;

    // We would ideally like to drive the car at this speed
    const double TARGET_VELOCITY = 70;

    const size_t STATE_SIZE = 6;
    const size_t X_STATE_OFFSET = 0;
    const size_t Y_STATE_OFFSET = 1;
    const size_t PSI_STATE_OFFSET = 2;
    const size_t VEL_STATE_OFFSET = 3;
    const size_t CTE_STATE_OFFSET = 4;
    const size_t EPSI_STATE_OFFSET = 5;

    /**
     * Offsets into each 8-element segment of the VARS vector
     */
    const size_t X_OFFSET = 0;
    const size_t Y_OFFSET = X_OFFSET + N;
    const size_t PSI_OFFSET = Y_OFFSET + N;
    const size_t VEL_OFFSET = PSI_OFFSET + N;
    const size_t CTE_OFFSET = VEL_OFFSET + N;
    const size_t EPSI_OFFSET = CTE_OFFSET + N;
    const size_t STEER_CTRL_OFFSET = EPSI_OFFSET + N;
    const size_t THROTTLE_CTRL_OFFSET = STEER_CTRL_OFFSET + N - 1;
}
#endif //MPC_CONSTANTS_H
