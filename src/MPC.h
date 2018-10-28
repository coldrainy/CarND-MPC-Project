#ifndef MPC_H
#define MPC_H
#pragma once
#include <vector>
#include "Eigen-3.3/Eigen/Core"


using namespace std;

  const size_t N = 30;
  const double dt = 0.05;
const double Lf = 2.67;

  const size_t x_start = 0;
  const size_t y_start = x_start + N;
  const size_t psi_start = y_start + N;
  const size_t v_start = psi_start + N;
  const size_t cte_start = v_start + N;
  const size_t epsi_start = cte_start + N;
  const size_t delta_start = epsi_start + N;
  const size_t a_start = delta_start + N - 1;

class MPC {
 public:
  MPC();

  virtual ~MPC();


  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
