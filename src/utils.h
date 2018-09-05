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

class Timer {
 public:
  typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> TimePoint;
  typedef std::chrono::nanoseconds Duration;

  static std::chrono::milliseconds ToMilliseconds(Duration duration) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }

  Timer() : iterations(0) {}
  ~Timer() {}

  TimePoint Start() { return Now(); }

  Duration Eval(TimePoint start) {
    auto end = Now();
    auto elapsed = end - start;
    elapsed_total += elapsed;
    ++iterations;
    return elapsed;
  }

  Duration AverageDuration() { return elapsed_total / iterations; }
  Duration TotalDuration() { return elapsed_total; }

 private:
  Duration elapsed_total{std::chrono::nanoseconds::zero()};
  size_t iterations = 0;

  inline TimePoint Now() { return std::chrono::steady_clock::now(); }
};

double Deg2rad(double x) { return x * M_PI / 180; }

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

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

}  // namespace MPC

#endif