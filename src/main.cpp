#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "utils.h"
#include "Constants.h"

// for convenience
using json = nlohmann::json;
using namespace Utilities;
using namespace Constants;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          /***************************************************
           * Read the data from the telemetry object
           *    j[1] is the data JSON object
           **************************************************/
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**************************************************
           * Transform waypoints to be from the car's perspective
           * this means we can consider px = 0, py = 0, and psi = 0
           * greatly simplifying future calculations
          **************************************************/
          vector<double> waypoints_x(ptsx.size());
          vector<double> waypoints_y(ptsx.size());

          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x[i] = (dx * cos(-psi) - dy * sin(-psi));
            waypoints_y[i] = (dx * sin(-psi) + dy * cos(-psi));
          }

          Eigen::Map<Eigen::VectorXd> waypoints_x_eig(&waypoints_x[0], STATE_SIZE);
          Eigen::Map<Eigen::VectorXd> waypoints_y_eig(&waypoints_y[0], STATE_SIZE);

          /*************************************************
          * Calculate steering angle and throttle using MPC.
          * Both are supposed to be between [-1, 1], so divide by deg2rad(25) before sending the steering value back.
          * Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          **************************************************/
          json msgJson;
          auto waypoint_coefficients = polyfit(waypoints_x_eig, waypoints_y_eig, POLYNOMIAL_ORDER);
          double cte = polyeval(waypoint_coefficients, 0);  // px = 0, py = 0
          double epsi = -atan(waypoint_coefficients[1]);  // p
          Eigen::VectorXd state(STATE_SIZE);
          state << 0, 0, 0, v, cte, epsi;
          auto solution = mpc.Solve(state, waypoint_coefficients);
          double steer_value = solution[0];
          double throttle_value = solution[1];
          msgJson["steering_angle"] = steer_value / (deg2rad(25));
          msgJson["throttle"] = throttle_value;

          /*************************************************
           * Display the MPC predicted trajectory:
           * Points (x,y) are in reference to the vehicle's coordinate system
           * and will be shown as a GREEN line in the simulator
           **************************************************/
          vector<double> mpc_x_vals(solution.size() / 2 - 1);
          vector<double> mpc_y_vals(solution.size() / 2 - 1);
          // Collect all the xs:
          for (int i = 2; i < solution.size(); i += 2) {
            mpc_x_vals[(i / 2) - 1] = (solution[i]);
          }
          // Collect all the ys:
          for (int j = 3; j < solution.size(); j += 2) {
            mpc_y_vals[(j / 2) - 1] = (solution[j]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          /*************************************************
           * Display the waypoints/reference line
           * Points (x,y) are in reference to the vehicle's coordinate system
           * and will be shown as a YELLO line in the simulator
          **************************************************/
          vector<double> next_x_vals(30);
          vector<double> next_y_vals(30);
          for (int i = 0; i < next_x_vals.size(); i++) {
            double x_value = i * POLYNOMIAL_ORDER;
            next_x_vals[i] = x_value;
            next_y_vals[i] = polyeval(waypoint_coefficients, x_value);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
