#include "MPC.h"
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen/Core"

namespace MPC {

class FG_eval {
 public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

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
      cost += W_CTE * CppAD::pow(vars[config.cte_start + t], 2);
      cost += W_EPSI * CppAD::pow(vars[config.epsi_start + t], 2);
      cost += W_V * CppAD::pow(vars[config.v_start + t] - config.target_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < config.steps_n - 1; ++t) {
      cost += W_DELTA * CppAD::pow(vars[config.delta_start + t], 2);
      cost += W_A * CppAD::pow(vars[config.a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < config.steps_n - 2; ++t) {
      cost += W_DELTA_D * CppAD::pow(vars[config.delta_start + t + 1] - vars[config.delta_start + t], 2);
      cost += W_A_D * CppAD::pow(vars[config.a_start + t + 1] - vars[config.a_start + t], 2);
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
    for (int t = 0; t < config.steps_n - 1; ++t) {
      // The state at time t.
      CppAD::AD<double> x0 = vars[config.x_start + t];
      CppAD::AD<double> y0 = vars[config.y_start + t];
      CppAD::AD<double> psi0 = vars[config.psi_start + t];
      CppAD::AD<double> v0 = vars[config.v_start + t];
      CppAD::AD<double> cte0 = vars[config.cte_start + t];
      CppAD::AD<double> epsi0 = vars[config.epsi_start + t];

      // The state at time t+1 .
      CppAD::AD<double> x1 = vars[config.x_start + t + 1];
      CppAD::AD<double> y1 = vars[config.y_start + t + 1];
      CppAD::AD<double> psi1 = vars[config.psi_start + t + 1];
      CppAD::AD<double> v1 = vars[config.v_start + t + 1];
      CppAD::AD<double> cte1 = vars[config.cte_start + t + 1];
      CppAD::AD<double> epsi1 = vars[config.epsi_start + t + 1];

      // Only consider the actuation at time t.
      CppAD::AD<double> delta0 = vars[config.delta_start + t - 1];
      CppAD::AD<double> a0 = vars[config.a_start + t - 1];

      // y = ax^3 + bx^2 + cx + d
      CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      // f'(y) = 2ax^2 + bx + c
      CppAD::AD<double> psides0 = CppAD::atan(coeffs[1] + coeffs[2] * x0 + 2 * coeffs[3] * CppAD::pow(x0, 2));

      // fg[0] contains the cost, fg[1] is current state
      fg[config.x_start + t + 2] = x1 - (x0 + v0 * CppAD::cos(psi0) * config.step_dt);
      fg[config.y_start + t + 2] = y1 - (y0 + v0 * CppAD::sin(psi0) * config.step_dt);
      fg[config.psi_start + t + 2] = psi1 - (psi0 + v0 * delta0 / LF * config.step_dt);
      fg[config.v_start + t + 2] = v1 - (v0 + a0 * config.step_dt);
      fg[config.cte_start + t + 2] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * config.step_dt));
      fg[config.epsi_start + t + 2] = epsi1 - ((psi0 - psides0) + v0 * delta0 / LF * config.step_dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() { MPC(10, 0.1, 50); }
MPC::MPC(size_t steps_n, size_t step_dt, double target_v) {
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

  // Set the number of model variables (includes both states and inputs).
  size_t vars_n = steps_n * STATE_SIZE + 2 * (steps_n - 1);
  // Set the number of constraints
  size_t constrains_n = steps_n * STATE_SIZE;

  vars = (vars_n);
  vars_lowerbound = (vars_n);
  vars_upperbound = (vars_n);
  constraints_lowerbound = (constrains_n);
  constraints_upperbound = (constrains_n);

  size_t start_idx = 0;

  config = {steps_n,
            step_dt,
            target_v,
            start_idx,
            start_idx += steps_n,
            start_idx += steps_n,
            start_idx += steps_n,
            start_idx += steps_n,
            start_idx += steps_n,
            start_idx += steps_n,
            start_idx += steps_n - 1};
}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs) {
  // Initialize the model variables
  InitVariables(state);
  // Initialize the variables and constraints upper and lower limits
  InitLimits(state);

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

  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values.
  return {solution.x[config.delta_start], solution.x[config.a_start]};
}

void MPC::InitVariables(const Eigen::VectorXd& state) {
  // Initial value of the independent variables, all 0 besides initial state.

  for (size_t i = 0; i < vars.size(); ++i) {
    vars[i] = 0.0;
  }

  // Set initial state
  vars[config.x_start] = state[0];
  vars[config.y_start] = state[1];
  vars[config.psi_start] = state[2];
  vars[config.v_start] = state[3];
  vars[config.cte_start] = state[4];
  vars[config.epsi_start] = state[5];

  // for (auto i = 0; i < state.size(); ++i) {
  //   vars[i * steps_n] = state[i];
  // }
}

void MPC::InitLimits(const Eigen::VectorXd& state) {
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

  // for (auto i = 0; i < state.size(); ++i) {
  //   constraints_lowerbound[i * steps_n] = state[i];
  //   constraints_upperbound[i * steps_n] = state[i];
  // }
}
}  // namespace MPC
