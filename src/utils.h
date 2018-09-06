#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <chrono>
#include "Eigen/Core"
#include "Eigen/QR"

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

namespace MPC {

// Class to keep track of computation time
class Timer {
 public:
  typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> TimePoint;
  typedef std::chrono::nanoseconds Duration;

  static std::chrono::milliseconds ToMilliseconds(Duration duration) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }

  Timer() : elapsed_total(std::chrono::nanoseconds::zero()), iterations(0) {}
  ~Timer() {}

  TimePoint Start() { return Now(); }

  Duration Eval(TimePoint start) {
    auto end     = Now();
    auto elapsed = end - start;
    elapsed_total += elapsed;
    ++iterations;
    return elapsed;
  }

  Duration AverageDuration() {
    if (iterations == 0) {
      return std::chrono::nanoseconds::zero();
    }
    return elapsed_total / iterations;
  }

  Duration TotalDuration() { return elapsed_total; }

 private:
  Duration elapsed_total;
  size_t iterations;

  inline TimePoint Now() { return std::chrono::steady_clock::now(); }
};

/**
 * Converts from milliseconds to seconds
 */
double Ms2s(double x) { return x / 1000.0; }

/**
 * Convers from miles per hour to meters per second
 */
double Mph2ms(double x) { return x * 0.44704; }

/**
 * Converts degrees to radians
 */
double Deg2rad(double x) { return x * M_PI / 180; }

/**
 * Converts radians to degrees
 */
double Rad2deg(double x) { return x * 180 / M_PI; }

/*
 * Evaluate a polynomial.
 */
double PolyEval(const Eigen::VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

/*
 * Fits a polynomial.
 *
 * Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 *
 * @return coefficients
 */
Eigen::VectorXd PolyFit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q      = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/**
 * Loads a configuration from file
 */
CONFIG readConfig(const std::string& file_name) {
  std::cout << "Loading configuration from file: " << file_name << std::endl;
  using json = nlohmann::json;

  std::ifstream file_in(file_name);

  if (!file_in) {
    throw std::runtime_error("Could not open file " + file_name);
  }

  std::string config((std::istreambuf_iterator<char>(file_in)), (std::istreambuf_iterator<char>()));

  auto config_json = json::parse(config);

  size_t delay   = config_json["delay"];
  size_t steps_n = config_json["steps_n"];
  double step_dt = config_json["step_dt"];
  double speed   = config_json["speed"];

  auto weights = config_json["weights"];

  double w_cte     = weights["cte"];
  double w_epsi    = weights["epsi"];
  double w_v       = weights["v"];
  double w_delta   = weights["delta"];
  double w_a       = weights["a"];
  double w_delta_d = weights["delta_d"];
  double w_a_d     = weights["a_d"];

  return {delay, steps_n, Ms2s(step_dt), Mph2ms(speed), w_cte, w_epsi, w_v, w_delta, w_a, w_delta_d, w_a_d};
}

void UpdateState(Eigen::VectorXd& state, double delta, double a, double Lf, double dt) {
  const double x    = state[0];
  const double y    = state[1];
  const double psi  = state[2];
  const double v    = state[3];
  const double cte  = state[4];
  const double epsi = state[5];

  state[0] = x + v * cos(psi) * dt;       // x t+1 = x + v * cos(psi) * dt
  state[1] = y + v * sin(psi) * dt;       // y t+1 = y + v * sin(psi) * dt
  state[2] = psi + v / Lf * delta * dt;   // psi t+1 = psi + v/LF * delta * dt
  state[3] = v + a * dt;                  // v t+1 = v + a * dt
  state[4] = cte + v * sin(epsi) * dt;    // cte t+1 = y - f(x) + v * sin(epsi) * dt
  state[5] = epsi + v / Lf * delta * dt;  // epsi t+1 = psi - psi_target + v/LF * delta * dt
}

}  // namespace MPC

#endif