#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "utils.h"

#define DEFAULT_DELAY 100
#define DEFAULT_STEPS 10
#define DEFAULT_STEPS_DT 0.1
#define DEFAULT_SPEED 100

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void SendMessage(uWS::WebSocket<uWS::SERVER> &ws, std::string msg) {
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void ProcessTelemetry(uWS::WebSocket<uWS::SERVER> &ws, MPC::MPC &mpc, MPC::Timer &timer, std::ofstream &file_out,
                      json &telemetry, size_t delay) {
  // json_obj[1] is the data JSON object
  const std::vector<double> ptsx = telemetry["ptsx"];
  const std::vector<double> ptsy = telemetry["ptsy"];

  const double px = telemetry["x"];
  const double py = telemetry["y"];
  const double psi = telemetry["psi"];
  const double v = telemetry["speed"];
  const double delta = telemetry["steering_angle"];
  const double a = telemetry["throttle"];

  // The waypoints are expressed in map coordinates, convert them to the car's coordinate system
  Eigen::VectorXd ptsx_car(ptsx.size());
  Eigen::VectorXd ptsy_car(ptsy.size());

  const double sin_npsi = sin(-psi);
  const double cos_npsi = cos(-psi);

  for (size_t i = 0; i < ptsx.size(); ++i) {
    const double delta_x = ptsx[i] - px;
    const double delta_y = ptsy[i] - py;

    ptsx_car[i] = delta_x * cos_npsi - delta_y * sin_npsi;
    ptsy_car[i] = delta_y * cos_npsi + delta_x * sin_npsi;
  }

  // Finds the 3rd defree polynomial coefficient for an "ideal curve"
  const auto coeffs = MPC::PolyFit(ptsx_car, ptsy_car, 3);

  // Computes the cross track error, which equals to the polynomial evaluated at 0
  // due to the coordinates conversion (y is 0)
  const double cte = coeffs[0];

  // Computes the orientation error, which can be found using the arctan of the derivative
  // of the polynomial: y = ax^3 + bx^2 + cx + d, y' = 3ax^2 + 2bx + c
  // Note, the evaluation is at x = 0 so only the 3rd coeffiecient c remains
  const double epsi = -atan(coeffs[1]);

  // Delta transformed
  const double delta_tr = -delta;

  // Accounts for delay, predicting the state at step t + 1
  const double dt = delay / 1000.0;
  const double pred_px = v * dt;                                // x t+1 = x + v * cos(psi) * dt
  const double pred_py = 0.0;                                   // y t+1 = y + v * sin(psi) * dt
  const double pred_psi = v / MPC::LF * delta_tr * dt;          // psi t+1 = psi + v/LF * delta * dt
  const double pred_v = v + a * dt;                             // v t+1 = v + a * dt
  const double pred_cte = cte + v * sin(epsi) * dt;             // cte t+1 = f(x) - y + v * sin(epsi) * dt
  const double pred_epsi = epsi + v / MPC::LF * delta_tr * dt;  // epsi t+1 = psi - psi_target + v/LF * delta * dt

  // Prepares the state vector
  Eigen::VectorXd state(MPC::STATE_SIZE);

  state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

  MPC::Timer::TimePoint start = timer.Start();
  // Solve for new actuations (and to show predicted x and y in the future)
  MPC::MPC_SOLUTION solution = mpc.Solve(state, coeffs);
  MPC::Timer::Duration duration = timer.Eval(start);

  // Next steering value, normalizing between [-1, 1] (taking into account center of gravity distance from the front)
  const double steer_value = solution.delta_next / (MPC::Deg2rad(25) * MPC::LF);
  // Next throttle value
  const double throttle_value = solution.a_next;

  std::cout << "Cost: " << std::setw(10) << solution.cost << " Steering: " << std::setw(12) << steer_value
            << " Throttle:" << std::setw(10) << throttle_value << std::endl;
  std::cout << "Time: " << std::setw(10) << MPC::Timer::ToMilliseconds(duration).count()
            << " ms, Average: " << MPC::Timer::ToMilliseconds(timer.AverageDuration()).count()
            << " ms, Total: " << MPC::Timer::ToMilliseconds(timer.TotalDuration()).count() << " ms" << std::endl;

  // Writes output to file
  file_out << px << "\t";
  file_out << py << "\t";
  file_out << psi << "\t";
  file_out << v << "\t";
  file_out << delta << "\t";
  file_out << a << "\t";
  file_out << delta << "\t";
  file_out << cte << "\t";
  file_out << epsi << "\t";
  file_out << steer_value << "\t";
  file_out << throttle_value << "\t";
  file_out << std::endl;
  file_out.flush();

  // Display the MPC predicted trajectory (green)
  const std::vector<double> mpc_x_vals = solution.x_predicted;
  const std::vector<double> mpc_y_vals = solution.y_predicted;

  // Display the waypoints/reference line (yellow)
  std::vector<double> next_x_vals(mpc.config.steps_n * 2);
  std::vector<double> next_y_vals(mpc.config.steps_n * 2);

  // X distance between points
  const double point_d = 4;

  for (size_t i = 1; i <= next_x_vals.size(); ++i) {
    next_x_vals[i - 1] = i * point_d;
    next_y_vals[i - 1] = MPC::PolyEval(coeffs, next_x_vals[i - 1]);
  }

  json msgJson;

  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle_value;
  msgJson["mpc_x"] = mpc_x_vals;
  msgJson["mpc_y"] = mpc_y_vals;
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  // Latency
  // The purpose is to mimic real driving conditions where
  // the car does actuate the commands instantly.
  //
  // Feel free to play around with this value but should be to drive
  // around the track with 100ms latency.
  //
  // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
  // SUBMITTING.
  if (delay > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  }
  SendMessage(ws, msg);
}

void RunServer(size_t delay, size_t steps, double steps_dt, double speed) {
  std::cout << "Delay: " << delay << " ms, Steps: " << steps << ", Steps Delta: " << steps_dt
            << " s, Target Speed: " << speed << std::endl;

  std::string file_name = "results.txt";
  // Creates a file to log values for plotting
  std::ofstream file_out(file_name);

  if (!file_out.is_open()) {
    std::cout << "Could not open file" << file_name << " for writing" << std::endl;
    exit(EXIT_FAILURE);
  }

  uWS::Hub h;
  MPC::MPC mpc = {steps, steps_dt, speed};
  MPC::Timer timer;

  h.onMessage(
      [&mpc, &timer, &file_out, &delay](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        std::string sdata = std::string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
          std::string message = HasData(sdata);
          if (message != "") {
            auto json_obj = json::parse(message);
            std::string event = json_obj[0].get<std::string>();
            if (event == "telemetry") {
              ProcessTelemetry(ws, mpc, timer, file_out, json_obj[1], delay);
            }
          } else {
            // Manual driving
            SendMessage(ws, "42[\"manual\",{}]");
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection(
      [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&file_out](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
    if (file_out.is_open()) {
      file_out.close();
    }
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    if (file_out.is_open()) {
      file_out.close();
    }
    exit(EXIT_FAILURE);
  }

  h.run();
}

template <typename Type>
Type ParseArg(int argc, char *argv[], std::string argName, std::istringstream &iss, int index, Type defaultValue) {
  Type out = defaultValue;
  if (argc > index) {
    iss.clear();
    iss.str(argv[index]);
    if (!(iss >> out)) {
      std::cout << "Could not read " << argName << ", using default: " << defaultValue << std::endl;
      out = defaultValue;
    }
  }
  return out;
}

int main(int argc, char *argv[]) {
  size_t delay, steps;
  double steps_dt, speed;

  std::istringstream iss;

  delay = ParseArg(argc, argv, "delay", iss, 1, DEFAULT_DELAY);
  steps = ParseArg(argc, argv, "steps", iss, 2, DEFAULT_STEPS);
  steps_dt = ParseArg(argc, argv, "steps delta", iss, 3, DEFAULT_STEPS_DT);
  speed = ParseArg(argc, argv, "speed", iss, 4, DEFAULT_SPEED);

  RunServer(delay, steps, steps_dt, speed);
}
