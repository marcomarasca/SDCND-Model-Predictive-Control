#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <vector>
#include "Eigen/Core"

namespace MPC {

typedef CPPAD_TESTVECTOR(double) Dvector;

// Number of state variables
const size_t STATE_SIZE = 6;

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
const double LF = 2.67;

// Weights for the components of the cost function
const double W_CTE = 2000;
const double W_EPSI = 2000;
const double W_V = 1;
const double W_DELTA = 50;
const double W_A = 50;
const double W_DELTA_D = 200;
const double W_A_D = 10;

// Limit bounds
const double L_MAX = 1.0e5;
const double L_THOTTLE = 1;
const double L_STEERING = 0.436332;

// Container for the MPC configuration
struct CONFIG {
  size_t steps_n;
  double step_dt;
  double target_speed;
  // The solver takes all the state variables and actuator variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;
};

// Container for the MPC solution
struct MPC_SOLUTION {
  double cost;
  // The values computed from the MPC
  double delta_next;
  double a_next;
  std::vector<double> x_predicted;
  std::vector<double> y_predicted;
};

class MPC {
 public:
  // MPC configuration
  CONFIG config;
  /*
   * Default constructor, uses 10 steps and 0.1 as delta time
   */
  MPC();

  /*
   * Contrsuctor
   *
   * @param steps_n Number of total steps to solve for
   * @params step_dt Delta time between each step (in seconds)
   * @params target_speed Target speed
   */
  MPC(size_t steps_n, double step_dt, double target_speed);

  virtual ~MPC();

  /*
   * Solve the model given an initial state and polynomial coefficients.
   *
   * @return The (first) actuator values (delta and a) as well as the points for the predicted trajectory (x_predicted
   * and y_predicted).
   */
  MPC_SOLUTION Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);

 private:
  const char* ipopt_options;

  // Variables for the solver
  Dvector vars;

  // lower and upper limits for variables
  Dvector vars_lowerbound;
  Dvector vars_upperbound;

  // lower and upper limits for contraints
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  /*
   * Variables and constraints initialization
   */
  void InitVariables();

  /*
   * Updates the variables and constraints limits according to the given state vector
   *
   * @param state State vector
   */
  void UpdateVariables(const Eigen::VectorXd& state);
};

}  // namespace MPC

#endif /* MPC_H */
