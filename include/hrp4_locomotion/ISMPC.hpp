#ifndef LABROB_ISMPC_HPP_
#define LABROB_ISMPC_HPP_

// STL
#include <fstream>
#include <memory>
#include <vector>

#include <hrp4_locomotion/LIPState.hpp>
#include <hrp4_locomotion/WalkingData.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace labrob {

class ISMPC{
 public:
  ISMPC(
      int64_t prediction_horizon_msec,
      int64_t mpc_timestep_msec,
      double omega,
      double foot_constraint_square_length,
      double foot_constraint_square_width
  );

  void solve(
      int64_t time,
      const labrob::WalkingData& walking_data,
      const labrob::LIPState& state
  );

  const Eigen::Vector3d& getInput() const;
  
  double getOmega() const;

 private:
  // NOTE: std::clamp available from C++17
  double clamp(double n, double n_min, double n_max);

  // Constant parameters:
  int num_variables_;
  int num_equality_constraints_;
  int num_inequality_constraints_;
  int N_;

  int64_t mpc_timestep_msec_;
  int64_t control_timestep_msec_;
  double omega_;
  double foot_constraint_square_length_;
  double foot_constraint_square_width_;

  // Matrices for prediction:
  Eigen::VectorXd p_;
  Eigen::MatrixXd P_;

  // Matrices for cost function:
  Eigen::MatrixXd cost_function_H_;
  Eigen::VectorXd cost_function_f_;
  double beta_ = 10000.0;

  // Matrices for stability constraint:
  Eigen::MatrixXd A_eq_;
  Eigen::VectorXd b_eq_;

  //Matrices for balance constraint:
  Eigen::MatrixXd A_zmp_;
  Eigen::VectorXd b_zmp_max_;
  Eigen::VectorXd b_zmp_min_;

  Eigen::Vector3d input_;

  // QP solver:
  std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> qp_solver_ptr_;

  std::ofstream pred_log_file_;

}; // end class ISMPC

} // end namespace labrob

#endif // LABROB_ISMPC_HPP_