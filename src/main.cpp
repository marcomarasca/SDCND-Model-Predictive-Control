#include <math.h>
#include <uWS/uWS.h>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "utils.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1         = s.find_first_of("[");
  auto b2         = s.rfind("}]");
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
                      json &telemetry) {
  // json_obj[1] is the data JSON object
  const std::vector<double> ptsx = telemetry["ptsx"];
  const std::vector<double> ptsy = telemetry["ptsy"];

  const double px    = telemetry["x"];
  const double py    = telemetry["y"];
  const double psi   = telemetry["psi"];
  const double v     = MPC::Mph2ms(telemetry["speed"]);  // Converts the speed from mph to m/s
  const double delta = telemetry["steering_angle"];
  const double a     = telemetry["throttle"];

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
  // due to the coordinates conversion (x is 0 and y is 0):
  // f(x) = ax^3 + bx^2 + cx + d
  // cte = y - f(x)
  const double cte = -coeffs[0];

  // Computes the orientation error, which can be found using the arctan of the derivative
  // of the polynomial:
  // y = ax^3 + bx^2 + cx + d
  // y' = 3ax^2 + 2bx + c
  // epsi = psi - arctan(y')
  // Note, the evaluation is at x = 0 so only the 3rd coeffiecient c remains
  const double epsi = -atan(coeffs[1]);

  // Prepares the state vector
  Eigen::VectorXd state(MPC::STATE_SIZE);

  // After the conversion to the car's coordinates x, y and psi are zero (e.g. oriented with the car)
  state << 0, 0, 0, v, cte, epsi;

  // Computes delta_t as the actuators delay + the computation delay
  const auto dt = MPC::Ms2s(mpc.config.delay + MPC::Timer::ToMilliseconds(timer.AverageDuration()).count());

  // Accounts for delay (including the computation delay), predicting the state at step t + 1
  MPC::UpdateState(state, -delta, a, MPC::LF, dt);

  MPC::Timer::TimePoint start = timer.Start();

  // Solve for new actuations (and to show predicted x and y in the future)
  MPC::MPC_SOLUTION solution = mpc.Solve(state, coeffs);

  MPC::Timer::Duration duration = timer.Eval(start);

  // Next steering value, normalizing between [-1, 1]
  const double steer_value = -solution.delta_next / MPC::L_STEERING;

  // Next throttle value
  const double throttle_value = solution.a_next;

  std::cout << std::setw(5) << "Cost:" << std::setw(10) << solution.cost << std::setw(9) << "CTE:" << std::setw(10)
            << cte << std::setw(10) << "Steering:" << std::setw(12) << steer_value << std::setw(10)
            << "Throttle:" << std::setw(6) << throttle_value << std::endl;

  std::cout << std::setw(5) << "Time:" << std::setw(7) << MPC::Timer::ToMilliseconds(duration).count() << " ms"
            << std::setw(9) << "Average:" << std::setw(7) << MPC::Timer::ToMilliseconds(timer.AverageDuration()).count()
            << " ms" << std::setw(10) << "Total:" << std::setw(9)
            << MPC::Timer::ToMilliseconds(timer.TotalDuration()).count() << " ms" << std::endl;

  // Writes output to file
  file_out << px << "\t";
  file_out << py << "\t";
  file_out << psi << "\t";
  file_out << v << "\t";
  file_out << delta << "\t";
  file_out << a << "\t";
  file_out << cte << "\t";
  file_out << epsi << "\t";
  file_out << steer_value << "\t";
  file_out << throttle_value << std::endl;
  file_out.flush();

  // Display the MPC predicted trajectory (green)
  const std::vector<double> mpc_x_vals = solution.x_predicted;
  const std::vector<double> mpc_y_vals = solution.y_predicted;

  // Display the waypoints/reference line (yellow)
  std::vector<double> next_x_vals(mpc.config.steps_n * 2);
  std::vector<double> next_y_vals(mpc.config.steps_n * 2);

  // X distance between points
  const double point_d = 2.5;

  for (size_t i = 1; i <= next_x_vals.size(); ++i) {
    next_x_vals[i - 1] = i * point_d;
    next_y_vals[i - 1] = MPC::PolyEval(coeffs, next_x_vals[i - 1]);
  }

  json msgJson;

  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"]       = throttle_value;
  msgJson["mpc_x"]          = mpc_x_vals;
  msgJson["mpc_y"]          = mpc_y_vals;
  msgJson["next_x"]         = next_x_vals;
  msgJson["next_y"]         = next_y_vals;

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
  if (mpc.config.delay > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(mpc.config.delay));
  }
  SendMessage(ws, msg);
}

void RunServer(MPC::CONFIG &config) {
  std::cout << "Delay: " << config.delay << " ms, Steps: " << config.steps_n << ", Steps Delta: " << config.step_dt
            << " s, Target Speed: " << config.target_speed << " m/s" << std::endl;

  std::string file_name = "results.txt";
  // Creates a file to log values for plotting
  std::ofstream file_out;

  uWS::Hub h;
  // Initialize MPC converting units
  MPC::MPC mpc = {config};
  MPC::Timer timer;

  h.onMessage([&mpc, &timer, &file_out](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string message = HasData(sdata);
      if (message != "") {
        auto json_obj     = json::parse(message);
        std::string event = json_obj[0].get<std::string>();
        if (event == "telemetry") {
          ProcessTelemetry(ws, mpc, timer, file_out, json_obj[1]);
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

  h.onConnection([&file_out, &file_name](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    file_out.open(file_name);
    if (!file_out.is_open()) {
      std::cout << "Could not open file " + file_name + " for writing" << std::endl;
    }
  });

  h.onDisconnection([&file_out](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

int main(int argc, char *argv[]) {
  // Default configuration
  MPC::CONFIG config = {100, 15, MPC::Ms2s(80), MPC::Mph2ms(70), 1.0, 50.0, 0.5, 1000.0, 1.0, 5000.0, 500.0};

  if (argc > 1) {
    std::string file_name = argv[1];

    try {
      config = MPC::readConfig(file_name);
    } catch (std::exception &e) {
      std::cout << e.what() << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  RunServer(config);
}
