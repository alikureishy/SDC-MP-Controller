# Model Predictive Controller (Autonomous Vehicles)


## Overview

This project aims to build a Model Predictive Controller that allows a simulated car to navigate around a racetrack, when provided with the following information at each time step:
- Vehicle's current state (possibly obtained from a state estimator located elsewhere)
- A planned trajectory for the vehicle to follow

It is assumed that the calculation of this state, and the waypoints, is performed elsewhere (in this case, in the simulator), and the only code this project will concern itself with is for the MPC controller that sends the steering and throttle actuation signals back to the simulated car, provided the aforementioned information at every step.

## Model Predictive Control overview

This control is more powerful and adaptive than the PID control, because it is built on the actual model of the car, which could either be a kinematic model, or a more nuanced dynamic model. Whereas the PID controller treats the plant model as a black box, and adjusts the controls based purely on feedback it receives from the plant, the MPC controller can utilize a model of the plant enables it to perform nuanced predictions of the vehicle in the future, and to base its actuations on those predictions. However, whereas a PID controller can be utilized universally just by tweaking the gain settings, a MPC must be modeled around the specific plant, and cannot be applied to a plant that is governed by a different kinematic (or dynamic) model.

## Implementation

The telemetry data that is received by the MPC, from the vehicle's sensors (i.e, from the simulator), is as follows:

```
| <Global Xs for waypoints> ("ptsx") | <Global Ys for waypoints> ("ptsy") | Current-X("x") | Current-Y ("y") | Orientation ("psi") | Velocity ("speed") | Steering Angle ("steering_angle")| Throttle ("throttle") |
```

The controller does the following at each timestep that telemetry data is received:
1- Receives telemetry data for timestep t
2- Fits the waypoints to a 3rd order polynomial
3- Uses its current state as the starting state
4- It optimizes the controls needed for the next 'N' steps, given various contraints that represent the model
5- Actuates the first control vector (discarding the remaining N-1 controls)
6- Repeat steps 1-5

### Model equations



### Sample time (dt) selection

Assuming a rise time (time it takes for a response to rise from 10% to 90% of the steady-state response of the plant) of approximately 1 second, the rule of thumb is to select a sample time that is about 5%-10% of that rise time, which (at 10%) is 0.1 seconds (100 ms).
 
### Prediction horizon (N) selection

The rule of thumb is for the prediction horizon to be 8-10 samples. I have chosen 8 here, in the interest of reducing computational load. (See future improvements item # 3 for alternate possibility).

### Polynomial fitting

Before being fed into the MPC, the waypoints are fitted to a 3rd degree polynomial. We use a degree-3 polynomial because it can accurately represent waypoints across most roads in the US.

### Handling actuation latency

A 100 ms actuation latency is simulated in this project. When the vehicle receives the telemetry data for each time step, it compensates for the latency by applying the MPC to state of the vehicle that is 100 ms in the future (using the model equations).  

## Future improvements

1. Implementing a control horizon of 2-4 samples (instead of the full length of the prediction horizon)
2. More nuanced cost function
3. A deep learning model that is trained to pick the optimal control for a given CTE & EPSI combination. This model would then linearly hop through the next N timesteps, determining at each step the optimal control to choose and the state of the car at the next step. This would achieve higher performance than the present model that does a gradient descent optimization at each time step. 
4. A mechanism wherein teh controller can adapt its latency compensation to suit the needs of the environment, without needing to hardcode it (as has been done in this project)   

## Appendix A - Installation & Dependencies

### Dependencies
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

### Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

### Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.
