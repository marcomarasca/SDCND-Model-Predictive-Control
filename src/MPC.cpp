#include "MPC.h"
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen/Core"

namespace MPC {

class FG_eval {
 public:
  typedef CppAD::AD<double> ADdouble;
  typedef CPPAD_TESTVECTOR(ADdouble) ADvector;

  FG_eval(const CONFIG& config, VAR_IDX& var_idx, const Eigen::VectorXd& coeffs)
      : config(config), var_idx(var_idx), coeffs(coeffs) {}

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
  // Variable indexes
  VAR_IDX var_idx;
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  void InitCost(ADdouble& cost, const ADvector& vars) {
    cost = 0.0;
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < config.steps_n; ++t) {
      const auto cte     = vars[var_idx.cte_start + t];
      const auto epsi    = vars[var_idx.epsi_start + t];
      const auto delta_v = vars[var_idx.v_start + t] - config.target_speed;

      cost += config.w_cte * CppAD::pow(cte, 2);
      cost += config.w_epsi * CppAD::pow(epsi, 2);
      cost += config.w_v * CppAD::pow(delta_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < config.steps_n - 1; ++t) {
      const auto delta = vars[var_idx.delta_start + t];
      const auto a     = vars[var_idx.a_start + t];

      cost += config.w_delta * CppAD::pow(delta, 2);
      cost += config.w_a * CppAD::pow(a, 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < config.steps_n - 2; ++t) {
      const auto delta_d = vars[var_idx.delta_start + t + 1] - vars[var_idx.delta_start + t];
      const auto a_d     = vars[var_idx.a_start + t + 1] - vars[var_idx.a_start + t];

      cost += config.w_delta_d * CppAD::pow(delta_d, 2);
      cost += config.w_a_d * CppAD::pow(a_d, 2);
    }
  }

  void InitConstraints(ADvector& fg, const ADvector& vars) {
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    fg[var_idx.x_start + 1]    = vars[var_idx.x_start];
    fg[var_idx.y_start + 1]    = vars[var_idx.y_start];
    fg[var_idx.psi_start + 1]  = vars[var_idx.psi_start];
    fg[var_idx.v_start + 1]    = vars[var_idx.v_start];
    fg[var_idx.cte_start + 1]  = vars[var_idx.cte_start];
    fg[var_idx.epsi_start + 1] = vars[var_idx.epsi_start];

    // The rest of the constraints
    for (size_t t = 0; t < config.steps_n - 1; ++t) {
      // The state at time t.
      const auto x_0     = vars[var_idx.x_start + t];
      const auto y_0     = vars[var_idx.y_start + t];
      const auto psi_0   = vars[var_idx.psi_start + t];
      const auto v_0     = vars[var_idx.v_start + t];
      const auto epsi_0  = vars[var_idx.epsi_start + t];
      const auto delta_0 = vars[var_idx.delta_start + t];
      const auto a_0     = vars[var_idx.a_start + t];

      // The state at time t+1 .
      const auto x_1    = vars[var_idx.x_start + t + 1];
      const auto y_1    = vars[var_idx.y_start + t + 1];
      const auto psi_1  = vars[var_idx.psi_start + t + 1];
      const auto v_1    = vars[var_idx.v_start + t + 1];
      const auto cte_1  = vars[var_idx.cte_start + t + 1];
      const auto epsi_1 = vars[var_idx.epsi_start + t + 1];

      const auto x_0_2 = CppAD::pow(x_0, 2);
      const auto x_0_3 = x_0_2 * x_0;

      // y = ax^3 + bx^2 + cx + d, f'(y) = 3ax^2 + bx + c
      const auto y_des_0   = coeffs[3] * x_0_3 + coeffs[2] * x_0_2 + coeffs[1] * x_0 + coeffs[0];
      const auto psi_des_0 = CppAD::atan(3.0 * coeffs[3] * x_0_2 + 2.0 * coeffs[2] * x_0 + coeffs[1]);

      // Model equations:
      //
      // x_[t+1]   = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1]   = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1]   = v[t] + a[t] * dt
      // cte[t+1]  = (y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = (psi[t] - psi_des[t]) + v[t] * delta[t] / Lf * dt
      const auto x_1_exp    = x_0 + v_0 * CppAD::cos(psi_0) * config.step_dt;
      const auto y_1_exp    = y_0 + v_0 * CppAD::sin(psi_0) * config.step_dt;
      const auto psi_1_exp  = psi_0 + v_0 / LF * delta_0 * config.step_dt;
      const auto v_1_exp    = v_0 + a_0 * config.step_dt;
      const auto cte_1_exp  = y_0 - y_des_0 + v_0 * CppAD::sin(epsi_0) * config.step_dt;
      const auto epsi_1_exp = psi_0 - psi_des_0 + v_0 / LF * delta_0 * config.step_dt;

      // Contraints the delta between variable at t+1 to be to the model expected value, note that the contraints for
      // all the variables is 0
      //
      // Note: fg[0] contains the cost, fg[1] is current state hence the +2
      fg[var_idx.x_start + t + 2]    = x_1 - x_1_exp;
      fg[var_idx.y_start + t + 2]    = y_1 - y_1_exp;
      fg[var_idx.psi_start + t + 2]  = psi_1 - psi_1_exp;
      fg[var_idx.v_start + t + 2]    = v_1 - v_1_exp;
      fg[var_idx.cte_start + t + 2]  = cte_1 - cte_1_exp;
      fg[var_idx.epsi_start + t + 2] = epsi_1 - epsi_1_exp;
    }
  }
};

MPC::MPC(CONFIG& config_in) : config(config_in), var_idx(config_in.steps_n) {
  // Set the number of model variables (includes both states and inputs) and constrints
  size_t vars_n       = config.steps_n * STATE_SIZE + 2 * (config.steps_n - 1);
  size_t constrains_n = config.steps_n * STATE_SIZE;

  // Set correct size
  vars.resize(vars_n);
  vars_lowerbound.resize(vars_n);
  vars_upperbound.resize(vars_n);
  constraints_lowerbound.resize(constrains_n);
  constraints_upperbound.resize(constrains_n);

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
  FG_eval fg_eval(config, var_idx, coeffs);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(ipopt_options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);

  // Check solution status
  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    std::cout << "[Warning] Could not find solution, status: " << solution.status << std::endl;
  }

  // Cost
  auto cost = solution.obj_value;

  // Sets the actuator values
  double delta_next = solution.x[var_idx.delta_start];
  double a_next     = solution.x[var_idx.a_start];

  std::vector<double> x_predicted(config.steps_n);
  std::vector<double> y_predicted(config.steps_n);

  // Sets the predicted trajectory points
  for (size_t i = 1; i < config.steps_n; ++i) {
    x_predicted[i] = solution.x[var_idx.x_start + i];
    y_predicted[i] = solution.x[var_idx.y_start + i];
  }

  return {cost, delta_next, a_next, x_predicted, y_predicted};
}

void MPC::InitVariables() {
  // Initial value of the independent variables
  for (size_t i = 0; i < vars.size(); ++i) {
    vars[i] = 0.0;
  }
  // Set all non-actuators upper and lowerlimits to the max negative and positive values.
  for (size_t i = 0; i < var_idx.delta_start; i++) {
    vars_lowerbound[i] = -L_MAX;
    vars_upperbound[i] = L_MAX;
  }

  // The upper and lower limits for steering
  for (size_t i = var_idx.delta_start; i < var_idx.a_start; ++i) {
    vars_lowerbound[i] = -L_STEERING;
    vars_upperbound[i] = L_STEERING;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = var_idx.a_start; i < vars.size(); ++i) {
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
  vars[var_idx.x_start]    = state[0];
  vars[var_idx.y_start]    = state[1];
  vars[var_idx.psi_start]  = state[2];
  vars[var_idx.v_start]    = state[3];
  vars[var_idx.cte_start]  = state[4];
  vars[var_idx.epsi_start] = state[5];

  constraints_lowerbound[var_idx.x_start]    = state[0];
  constraints_lowerbound[var_idx.y_start]    = state[1];
  constraints_lowerbound[var_idx.psi_start]  = state[2];
  constraints_lowerbound[var_idx.v_start]    = state[3];
  constraints_lowerbound[var_idx.cte_start]  = state[4];
  constraints_lowerbound[var_idx.epsi_start] = state[5];

  constraints_upperbound[var_idx.x_start]    = state[0];
  constraints_upperbound[var_idx.y_start]    = state[1];
  constraints_upperbound[var_idx.psi_start]  = state[2];
  constraints_upperbound[var_idx.v_start]    = state[3];
  constraints_upperbound[var_idx.cte_start]  = state[4];
  constraints_upperbound[var_idx.epsi_start] = state[5];
}
}  // namespace MPC
