#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <vector>
#include "Eigen/Core"

namespace MPC {

typedef CPPAD_TESTVECTOR(double) Dvector;

// Number of state variables
#define STATE_SIZE 6

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
#define LF 2.67

// Weights for the components of the cost function
#define W_CTE 1000
#define W_EPSI 1
#define W_V 1
#define W_DELTA 1
#define W_A 1
#define W_DELTA_D 1
#define W_A_D 1

// Limit bounds
#define L_MAX 1.0e19
#define L_THOTTLE 1
#define L_STEERING 0.436332

struct CONFIG {
  size_t steps_n;
  size_t step_dt;
  double target_v;
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

class MPC {
 public:
  /*
   * Default constructor, uses 10 steps and 0.1 as delta time
   */
  MPC();

  /*
   * Contrsuctor
   *
   * @param steps_n Number of total steps to solve for
   * @params step_dt Delta time between each step
   */
  MPC(size_t steps_n, size_t step_dt, double target_v);

  virtual ~MPC();

  /*
   * Solve the model given an initial state and polynomial coefficients.
   *
   * @return The first actuations.
   */
  std::vector<double> Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);

 private:
  const char* ipopt_options;

  CONFIG config;

  // Variables for the solver
  Dvector vars;

  // lower and upper limits for variables
  Dvector vars_lowerbound;
  Dvector vars_upperbound;

  // lower and upper limits for contraints
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  void InitVariables(const Eigen::VectorXd& state);
  void InitLimits(const Eigen::VectorXd& state);
};

}  // namespace MPC

#endif /* MPC_H */
