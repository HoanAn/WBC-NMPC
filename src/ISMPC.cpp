#include <hrp4_locomotion/ISMPC.hpp>
#include <chrono> 

namespace labrob {

ISMPC::ISMPC(
    int64_t prediction_horizon_msec,
    int64_t mpc_timestep_msec,
    double omega,
    double foot_constraint_square_length,
    double foot_constraint_square_width
) : mpc_timestep_msec_(mpc_timestep_msec),
    omega_(omega),
    foot_constraint_square_length_(foot_constraint_square_length),
    foot_constraint_square_width_(foot_constraint_square_width),
    input_(Eigen::Vector3d::Zero()) {

  double mpc_timestep = 0.001 * static_cast<double>(mpc_timestep_msec_);

  num_variables_ = 3 * prediction_horizon_msec / mpc_timestep_msec_;
  num_equality_constraints_ = 3;
  num_inequality_constraints_ = num_variables_;

  N_ = prediction_horizon_msec / mpc_timestep_msec;

  qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(
          num_variables_, num_equality_constraints_, num_inequality_constraints_
      )
  );

  // Setup matrices size:
  cost_function_H_ = Eigen::MatrixXd(num_variables_, num_variables_);
  cost_function_f_ = Eigen::VectorXd(num_variables_);

  A_eq_ = Eigen::MatrixXd(num_equality_constraints_, num_variables_);
  b_eq_ = Eigen::VectorXd(num_equality_constraints_);

  A_zmp_ = Eigen::MatrixXd(num_inequality_constraints_, num_variables_);
  b_zmp_max_ = Eigen::VectorXd(num_inequality_constraints_);
  b_zmp_min_ = Eigen::VectorXd(num_inequality_constraints_);

  p_ = Eigen::VectorXd::Ones(N_, 1);
  P_ = Eigen::MatrixXd::Constant(N_, N_, mpc_timestep);

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < N_; ++j){
      if (j > i) P_(i, j) = 0;
    }
  }

  A_zmp_.setZero();
  A_zmp_.block(     0,      0, N_, N_) = P_;
  A_zmp_.block(    N_,     N_, N_, N_) = P_;
  A_zmp_.block(2 * N_, 2 * N_, N_, N_) = P_;
}

void
ISMPC::solve(
    int64_t time,
    const labrob::WalkingData& walking_data,
    const labrob::LIPState& state
) {

  auto start_time = std::chrono::system_clock::now(); 

  double mpc_timestep = 0.001 * static_cast<double>(mpc_timestep_msec_);

  //const auto& feet_placement = walking_data.footstep_plan.front().getFeetPlacement();
  //Eigen::Vector3d p_support = feet_placement.getSupportFootConfiguration().p;
  //Eigen::Vector3d p_swing = feet_placement.getSwingFootConfiguration().p;

  Eigen::VectorXd mc_x(N_);
  Eigen::VectorXd mc_y(N_);
  Eigen::VectorXd mc_z(N_);

  // Set vector n_k to [d_0 / delta_mpc, d_1 / delta_mpc, d_2 / delta_mpc, ...],
  // where d_k is the duration of footstep plan element f_k, n_ini is the number
  // of MPC iterations already executed for current footstep plan element,
  // delta_mpc is the sampling time of the MPC.
  std::vector<int> n_k;
  for (const auto& footstep_plan_elem : walking_data.footstep_plan) {
    n_k.push_back(footstep_plan_elem.getDuration() / mpc_timestep_msec_);
  }
  int n_ini = (time - walking_data.t0) / mpc_timestep_msec_;

  int n = 0, k = 0;
  while (n < N_) {
    const auto& footstep_plan_elem = walking_data.footstep_plan[k];
    const auto& walking_state = walking_data.footstep_plan[k].getWalkingState();
    int n_bar = n_k[k];
    if (k == 0) n_bar -= n_ini;
    if (n + n_bar >= N_) n_bar = N_ - n;
    Eigen::MatrixXd mapping(n_bar, 2);
    if (walking_state == labrob::WalkingState::PostureRegulation ||
        walking_state == labrob::WalkingState::Standing) {
      for (int i = 0; i < mapping.rows(); ++i) {
        mapping(i, 0) = 0.5;
        mapping(i, 1) = 0.5;
      }
    } else if (walking_state == labrob::WalkingState::Starting) {
      for (int i = 0; i < mapping.rows(); ++i) {
        double s;
        if (k == 0) s = static_cast<double>(n_ini + i) / n_k[0];
        else s = static_cast<double>(i) / mapping.rows();
        mapping(i, 0) = 0.5 + 0.5 * s;
        mapping(i, 1) = 0.5 - 0.5 * s;
      }
    } else if (walking_state == labrob::WalkingState::SingleSupport) {
      for (int i = 0; i < mapping.rows(); ++i) {
        mapping(i, 0) = 1.0;
        mapping(i, 1) = 0.0;
      }
    } else if (walking_state == labrob::WalkingState::DoubleSupport) {
      for (int i = 0; i < mapping.rows(); ++i) {
        double s;
        if (k == 0) s = static_cast<double>(n_ini + i) / n_k[0];
        else s = static_cast<double>(i) / mapping.rows();
        mapping(i, 0) = (1.0 - s);
        mapping(i, 1) = s;
      }
    } else if (walking_state == labrob::WalkingState::Stopping) {
      for (int i = 0; i < mapping.rows(); ++i) {
        double s;
        if (k == 0) s = static_cast<double>(n_ini + i) / n_k[0];
        else s = static_cast<double>(i) / mapping.rows();
        mapping(i, 0) = 1.0 - 0.5 * s;
        mapping(i, 1) = 0.5 * s;
      }
    }
    const auto& p_support = footstep_plan_elem.getFeetPlacement().getSupportFootConfiguration().p;
    const auto& p_swing = footstep_plan_elem.getFeetPlacement().getSwingFootConfiguration().p;
    Eigen::VectorXd varying_x = mapping * Eigen::Vector2d(p_support.x(), p_swing.x());
    Eigen::VectorXd varying_y = mapping * Eigen::Vector2d(p_support.y(), p_swing.y());
    Eigen::VectorXd varying_z = mapping * Eigen::Vector2d(p_support.z(), p_swing.z());
    mc_x.segment(n, varying_x.rows()) = varying_x;
    mc_y.segment(n, varying_y.rows()) = varying_y;
    mc_z.segment(n, varying_z.rows()) = varying_z;

    // // print mc_x, mc_y and mc_z
    // std::cerr << "mc_x: " << mc_x.transpose() << std::endl;
    // std::cerr << "mc_y: " << mc_y.transpose() << std::endl;
    // std::cerr << "mc_z: " << mc_z.transpose() << std::endl;

    n += n_bar;
    ++k;
  }

  b_zmp_min_ << mc_x - Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_length_ / 2.0 + state.zmp_pos_(0)),
                mc_y - Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_width_ / 2.0 + state.zmp_pos_(1)),
                mc_z - Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_length_ / 2.0 + state.zmp_pos_(2));
  b_zmp_max_ << mc_x + Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_length_ / 2.0 - state.zmp_pos_(0)),
                mc_y + Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_width_ / 2.0 - state.zmp_pos_(1)),
                mc_z + Eigen::MatrixXd::Constant(N_, 1, foot_constraint_square_length_ / 2.0 - state.zmp_pos_(2));

  Eigen::VectorXd b(N_);
  A_eq_.setZero();

  for(int i = 0; i < N_; ++i){
    b(i) = std::pow(std::exp(-omega_ * mpc_timestep),i);
  }

  A_eq_.block(0,      0, 1, N_) = (1.0 / omega_) * (1.0 - std::exp(-omega_ * mpc_timestep))*b.transpose();
  A_eq_.block(1,     N_, 1, N_) = (1.0 / omega_) * (1.0 - std::exp(-omega_ * mpc_timestep))*b.transpose();
  A_eq_.block(2, 2 * N_, 1, N_) = (1.0 / omega_) * (1.0 - std::exp(-omega_ * mpc_timestep))*b.transpose();

  b_eq_ << state.com_pos_(0) + state.com_vel_(0) / omega_ - state.zmp_pos_(0),
      state.com_pos_(1) + state.com_vel_(1) / omega_ - state.zmp_pos_(1),
      state.com_pos_(2) + state.com_vel_(2) / omega_ - (state.zmp_pos_(2) + 9.81 / std::pow(omega_, 2.0));

  cost_function_H_.setZero();
  cost_function_H_.block(     0,      0, N_, N_) = Eigen::MatrixXd::Identity(N_, N_) + beta_ * P_.transpose() * P_;
  cost_function_H_.block(    N_,     N_, N_, N_) = Eigen::MatrixXd::Identity(N_, N_) + beta_ * P_.transpose() * P_;
  cost_function_H_.block(2 * N_, 2 * N_, N_, N_) = Eigen::MatrixXd::Identity(N_, N_) + beta_ * P_.transpose() * P_;

  cost_function_f_.block(     0, 0, N_, 1) = beta_ * P_.transpose() * (p_ * state.zmp_pos_.x() - mc_x);
  cost_function_f_.block(    N_, 0, N_, 1) = beta_ * P_.transpose() * (p_ * state.zmp_pos_.y() - mc_y);
  cost_function_f_.block(2 * N_, 0, N_, 1) = beta_ * P_.transpose() * (p_ * state.zmp_pos_.z() - mc_z);

  // Solve QP
  qp_solver_ptr_->solve(
      cost_function_H_,
      cost_function_f_,
      A_eq_,
      b_eq_,
      A_zmp_,
      b_zmp_min_,
      b_zmp_max_
  );
  auto decisionVariables = qp_solver_ptr_->get_solution();

  // Split the QP solution in ZMP dot and footsteps
  Eigen::VectorXd zDotOptimalX(N_);
  Eigen::VectorXd zDotOptimalY(N_);
  Eigen::VectorXd zDotOptimalZ(N_);

  zDotOptimalX = (decisionVariables.segment(     0, N_));
  zDotOptimalY = (decisionVariables.segment(    N_, N_));
  zDotOptimalZ = (decisionVariables.segment(2 * N_, N_));

  input_.x() = zDotOptimalX(0);
  input_.y() = zDotOptimalY(0);
  input_.z() = zDotOptimalZ(0);

  auto end_time = std::chrono::system_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  std::cerr << "ISMPC solve time: " << elapsed_time << " microsecondi" << std::endl;
}

const Eigen::Vector3d& ISMPC::getInput() const {
  return input_;
}

double
ISMPC::getOmega() const {
  return omega_;
}

double
ISMPC::clamp(double n, double n_min, double n_max) {
  return std::max(n_min, std::min(n_max, n));
}

} // end namespace labrob