#include <hrp4_locomotion/DiscreteLIPDynamics.hpp>
#include <hrp4_locomotion/utils.hpp>

namespace labrob {

DiscreteLIPDynamics::DiscreteLIPDynamics(double eta, int64_t timestep_msec)
  : timestep_(0.001 * static_cast<double>(timestep_msec)), eta_(eta) {
  double ch = cosh(eta * timestep_);
  double sh = sinh(eta * timestep_);
  A_ << ch, sh / eta, 1.0 - ch, eta * sh, ch, -eta * sh, 0.0, 0.0, 1.0;
  B_ << timestep_ - sh / eta, 1.0 - ch, timestep_;
}

LIPState
DiscreteLIPDynamics::integrate(const LIPState& lip_state, const LIPControlInput& zmp_vel, const Eigen::Vector3d& disturbance) {
  Eigen::Vector3d nextStateX = updateState(lip_state, zmp_vel.x(), 0);
  Eigen::Vector3d nextStateY = updateState(lip_state, zmp_vel.y(), 1);
  Eigen::Vector3d nextStateZ = updateState(lip_state, zmp_vel.z(), 2);
  return LIPState(
      Eigen::Vector3d(nextStateX(0), nextStateY(0) + timestep_ * disturbance.x(), nextStateZ(0)),
      Eigen::Vector3d(nextStateX(1), nextStateY(1) + timestep_ * disturbance.y(), nextStateZ(1)),
      Eigen::Vector3d(nextStateX(2), nextStateY(2) + timestep_ * disturbance.z(), nextStateZ(2))
  );
}

Eigen::Vector3d
DiscreteLIPDynamics::updateState(const LIPState& lip_state, double zmpDot, int dim) {
  double com_target_height = 9.81 / std::pow(eta_, 2.0);

  Eigen::Vector3d currentState = Eigen::Vector3d(
      lip_state.com_pos_(dim),
      lip_state.com_vel_(dim),
      lip_state.zmp_pos_(dim)
  );

  if (dim == 2) return A_ * (currentState + Eigen::Vector3d(0.0, 0.0, com_target_height)) + B_ * zmpDot - Eigen::Vector3d(0.0, 0.0, com_target_height);

  return A_ * currentState + B_ * zmpDot;
}

} // end namespace labrob