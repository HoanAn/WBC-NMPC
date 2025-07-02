#ifndef LABROB_DISCRETE_LIP_DYNAMICS
#define LABROB_DISCRETE_LIP_DYNAMICS

#include <hrp4_locomotion/LIPState.hpp>

namespace labrob {

typedef Eigen::Vector3d LIPControlInput;

class DiscreteLIPDynamics {
 public:
  DiscreteLIPDynamics(double eta, int64_t timestep_msec);
  LIPState integrate(const LIPState& lip_state, const LIPControlInput& zmp_vel, const Eigen::Vector3d& disturbance=Eigen::Vector3d::Zero());
 private:
  Eigen::Vector3d updateState(const LIPState& lip_state, double zmpDot, int dim);

  double timestep_;
  double eta_;
  Eigen::Matrix3d A_;
  Eigen::Vector3d B_;
};

} // end namespace labrob

#endif // LABROB_DISCRETE_LIP_DYNAMICS