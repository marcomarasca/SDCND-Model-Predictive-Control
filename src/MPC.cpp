#include "MPC.h"
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen/Core"

namespace MPC {

class FG_eval {
 public:
  typedef CppAD::AD<double> ADdouble;
  typedef CPPAD_TESTVECTOR(ADdouble) ADvector;

  FG_eval(const CONFIG& config, const Eigen::VectorXd& coeffs) : config(config), coeffs(coeffs) {}

  /**
   * @param fg is a vector containing the cost and constraints.
   * @param vars is a vector containing the variable values (state & actuators).
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    InitCost(fg[0], vars);
    InitConstraints(fg, vars);
  }

 private:
  // MPC configuration parameters
  CONFIG config;
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  void InitCost(CppAD::AD<double>& cost, const ADvector& vars) {
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < config.steps_n; ++t) {
      const ADdouble cte = vars[config.cte_start + t];
      const ADdouble epsi = vars[config.epsi_start + t];
      const ADdouble delta_v = vars[config.v_start + t] - config.target_speed;

      cost += W_CTE * cte * cte + W_EPSI * epsi * epsi + W_V * delta_v * delta_v;
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < config.steps_n - 1; ++t) {
      const ADdouble delta = vars[config.delta_start + t];
      const ADdouble a = vars[config.a_start + t];

      cost += W_DELTA * delta * delta + W_A * a * a;
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < config.steps_n - 2; ++t) {
      const ADdouble delta_d = vars[config.delta_start + t + 1] - vars[config.delta_start + t];
      const ADdouble a_d = vars[config.a_start + t + 1] - vars[config.a_start + t];

      cost += W_DELTA_D * delta_d * delta_d + W_A_D * a_d * a_d;
    }
  }

  void InitConstraints(ADvector& fg, const ADvector& vars) {
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    fg[config.x_start + 1] = vars[config.x_start];
    fg[config.y_start + 1] = vars[config.y_start];
    fg[config.psi_start + 1] = vars[config.psi_start];
    fg[config.v_start + 1] = vars[config.v_start];
    fg[config.cte_start + 1] = vars[config.cte_start];
    fg[config.epsi_start + 1] = vars[config.epsi_start];

    // The rest of the constraints
    for (size_t t = 0; t < config.steps_n - 1; ++t) {
      // The state at time t.
      const ADdouble x0 = vars[config.x_start + t];
      const ADdouble y0 = vars[config.y_start + t];
      const ADdouble psi0 = vars[config.psi_start + t];
      const ADdouble v0 = vars[config.v_start + t];
      const ADdouble cte0 = vars[config.cte_start + t];
      const ADdouble epsi0 = vars[config.epsi_start + t];
      const ADdouble delta0 = vars[config.delta_start + t];
      const ADdouble a0 = vars[config.a_start + t];

      // The state at time t+1 .
      const ADdouble x1 = vars[config.x_start + t + 1];
      const ADdouble y1 = vars[config.y_start + t + 1];
      const ADdouble psi1 = vars[config.psi_start + t + 1];
      const ADdouble v1 = vars[config.v_start + t + 1];
      const ADdouble cte1 = vars[config.cte_start + t + 1];
      const ADdouble epsi1 = vars[config.epsi_start + t + 1];

      // y = ax^3 + bx^2 + cx + d
      const ADdouble f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      // f'(y) = 3ax^2 + bx + c
      const ADdouble psides0 = CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * CppAD::pow(x0, 2));

      // fg[0] contains the cost, fg[1] is current state
      fg[config.x_start + t + 2] = x1 - (x0 + v0 * CppAD::cos(psi0) * config.step_dt);
      fg[config.y_start + t + 2] = y1 - (y0 + v0 * CppAD::sin(psi0) * config.step_dt);
      fg[config.psi_start + t + 2] = psi1 - (psi0 - v0 / LF * delta0 * config.step_dt);
      fg[config.v_start + t + 2] = v1 - (v0 + a0 * config.step_dt);
      fg[config.cte_start + t + 2] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * config.step_dt));
      fg[config.epsi_start + t + 2] = epsi1 - (psi0 - psides0 - v0 / LF * delta0 * config.step_dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() { MPC(10, 0.1, 50); }

MPC::MPC(size_t steps_n, double step_dt, double target_speed) {
  // Set the number of model variables (includes both states and inputs).
  size_t vars_n = steps_n * STATE_SIZE + 2 * (steps_n - 1);
  // Set the number of constraints
  size_t constrains_n = steps_n * STATE_SIZE;

  config = {steps_n, step_dt, target_speed};

  config.x_start = 0;
  config.y_start = config.x_start + steps_n;
  config.psi_start = config.y_start + steps_n;
  config.v_start = config.psi_start + steps_n;
  config.cte_start = config.v_start + steps_n;
  config.epsi_start = config.cte_start + steps_n;
  config.delta_start = config.epsi_start + steps_n;
  config.a_start = config.delta_start + steps_n - 1;

  vars = (vars_n);
  vars_lowerbound = (vars_n);
  vars_upperbound = (vars_n);
  constraints_lowerbound = (constrains_n);
  constraints_upperbound = (constrains_n);

  // Initializes the model to default values and constraints
  InitVariables();

  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  // Also, currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  ipopt_options =
      "Integer print_level  0\n"
      "Sparse  true         forward\n"
      "Sparse  true         reverse\n"
      "Numeric max_cpu_time 0.5\n";
}
MPC::~MPC() {}

MPC_SOLUTION MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs) {
  // Updates the variables and constraints from the input state vector
  UpdateVariables(state);

  // object that computes objective and constraints
  FG_eval fg_eval(config, coeffs);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(ipopt_options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);

  // TODO Check some of the solution values
  // bool ok = true;
  // ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;

  // Sets the actuator values
  double delta_next = solution.x[config.delta_start];
  double a_next = solution.x[config.a_start];

  std::vector<double> x_predicted(config.steps_n);
  std::vector<double> y_predicted(config.steps_n);

  // Sets the predicted trajectory points
  for (size_t i = 1; i < config.steps_n; ++i) {
    x_predicted[i] = solution.x[config.x_start + i];
    y_predicted[i] = solution.x[config.y_start + i];
  }

  return {cost, delta_next, a_next, x_predicted, y_predicted};
}

void MPC::InitVariables() {
  // Initial value of the independent variables
  for (size_t i = 0; i < vars.size(); ++i) {
    vars[i] = 0.0;
  }
  // Set all non-actuators upper and lowerlimits to the max negative and positive values.
  for (size_t i = 0; i < config.delta_start; i++) {
    vars_lowerbound[i] = -L_MAX;
    vars_upperbound[i] = L_MAX;
  }

  // The upper and lower limits for steering
  for (size_t i = config.delta_start; i < config.a_start; ++i) {
    vars_lowerbound[i] = -L_STEERING;
    vars_upperbound[i] = L_STEERING;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = config.a_start; i < vars.size(); ++i) {
    vars_lowerbound[i] = -L_THOTTLE;
    vars_upperbound[i] = L_THOTTLE;
  }
  // Lower and upper limits for the constraints, all 0 besides initial state.
  for (size_t i = 0; i < constraints_lowerbound.size(); ++i) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
}

void MPC::UpdateVariables(const Eigen::VectorXd& state) {
  // Set initial state
  vars[config.x_start] = state[0];
  vars[config.y_start] = state[1];
  vars[config.psi_start] = state[2];
  vars[config.v_start] = state[3];
  vars[config.cte_start] = state[4];
  vars[config.epsi_start] = state[5];

  constraints_lowerbound[config.x_start] = state[0];
  constraints_lowerbound[config.y_start] = state[1];
  constraints_lowerbound[config.psi_start] = state[2];
  constraints_lowerbound[config.v_start] = state[3];
  constraints_lowerbound[config.cte_start] = state[4];
  constraints_lowerbound[config.epsi_start] = state[5];

  constraints_upperbound[config.x_start] = state[0];
  constraints_upperbound[config.y_start] = state[1];
  constraints_upperbound[config.psi_start] = state[2];
  constraints_upperbound[config.v_start] = state[3];
  constraints_upperbound[config.cte_start] = state[4];
  constraints_upperbound[config.epsi_start] = state[5];
}
}  // namespace MPC
