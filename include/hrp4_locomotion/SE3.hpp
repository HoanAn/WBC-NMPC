#ifndef LABROB_SE3_HPP_
#define LABROB_SE3_HPP_

#include <Eigen/Core>

namespace labrob {

class SE3 {
 public:
  SE3();
  SE3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

  Eigen::Matrix3d R;
  Eigen::Vector3d p;
}; // end class SE3

} // end namespace labrob

#endif // LABROB_SE3_HPP_