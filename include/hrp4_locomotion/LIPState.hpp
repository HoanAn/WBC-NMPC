#ifndef LABROB_LIP_STATE_HPP_
#define LABROB_LIP_STATE_HPP_

#include <Eigen/Core>

namespace labrob {

class LIPState {
 public:
  LIPState() = default;
  LIPState(
      const Eigen::Vector3d& com_pos,
      const Eigen::Vector3d& com_vel,
      const Eigen::Vector3d& zmp_pos
  );

  Eigen::Vector3d com_pos_;
  Eigen::Vector3d com_vel_;
  Eigen::Vector3d zmp_pos_;
}; // end class LIPState

} // end namespace labrob

#endif // LABROB_LIP_STATE_HPP_