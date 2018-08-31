#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

namespace MPC {

class MPC {
 public:
  MPC();

  virtual ~MPC();

  /*
   * Solve the model given an initial state and polynomial coefficients.
   *
   * @return The first actuatotions.
   */
  std::vector<double> Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);
};

}  // namespace MPC

#endif /* MPC_H */
