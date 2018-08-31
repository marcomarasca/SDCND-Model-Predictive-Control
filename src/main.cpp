#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "utils.h"

#define DEFAULT_DELAY 100

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
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

void sendMessage(uWS::WebSocket<uWS::SERVER> &ws, std::string msg) {
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void processTelemetry(uWS::WebSocket<uWS::SERVER> &ws, MPC::MPC &mpc, json &telemetry, unsigned delay) {
  // json_obj[1] is the data JSON object
  std::vector<double> ptsx = telemetry[1]["ptsx"];
  std::vector<double> ptsy = telemetry[1]["ptsy"];
  double px = telemetry[1]["x"];
  double py = telemetry[1]["y"];
  double psi = telemetry[1]["psi"];
  double v = telemetry[1]["speed"];

  /*
   * TODO: Calculate steering angle and throttle using MPC.
   *
   * Both are in between [-1, 1].
   *
   */
  double steer_value;
  double throttle_value;

  json msgJson;
  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
  // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle_value;

  // Display the MPC predicted trajectory
  std::vector<double> mpc_x_vals;
  std::vector<double> mpc_y_vals;

  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
  // the points in the simulator are connected by a Green line

  msgJson["mpc_x"] = mpc_x_vals;
  msgJson["mpc_y"] = mpc_y_vals;

  // Display the waypoints/reference line
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
  // the points in the simulator are connected by a Yellow line

  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  std::cout << msg << std::endl;
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
  sendMessage(ws, msg);
}

void runServer(unsigned delay) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC::MPC mpc;

  h.onMessage([&mpc, &delay](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string message = hasData(sdata);
      if (message != "") {
        auto json_obj = json::parse(message);
        std::string event = json_obj[0].get<std::string>();
        if (event == "telemetry") {
          processTelemetry(ws, mpc, json_obj, delay);
        }
      } else {
        // Manual driving
        sendMessage(ws, "42[\"manual\",{}]");
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    exit(EXIT_FAILURE);
  }

  h.run();
}

int main(int argc, char *argv[]) {
  unsigned delay = DEFAULT_DELAY;

  if (argc > 1) {
    std::istringstream iss;

    iss.str(argv[1]);
    if (!(iss >> delay)) {
      std::cout << "Could not delay, using default: " << DEFAULT_DELAY << std::endl;
      delay = DEFAULT_DELAY;
    }
  }

  runServer(delay);
}
