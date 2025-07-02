#include <hrp4_locomotion/SE3.hpp>

namespace labrob {

SE3::SE3() : R(Eigen::Matrix3d::Identity()), p(Eigen::Vector3d::Zero()) { }
SE3::SE3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
  : R(rotation), p(translation) { }

} // end namespace labrob