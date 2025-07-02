#ifndef LABROB_HRP4_LOCOMOTION_GAIT_CONFIGURATION
#define LABROB_HRP4_LOCOMOTION_GAIT_CONFIGURATION

#include <Eigen/Core>

#include <hrp4_locomotion/JointState.hpp>
#include <hrp4_locomotion/SE3.hpp>

namespace labrob {
struct ee3 {
  Eigen::Vector3d pos; Eigen::Vector3d vel; Eigen::Vector3d acc;
};

struct ee_rot {
  Eigen::Matrix3d pos; Eigen::Vector3d vel; Eigen::Vector3d acc;
};

struct ee6 {
  labrob::SE3 pos; Eigen::Vector<double, 6> vel; Eigen::Vector<double, 6> acc;
};

struct GaitConfiguration {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Velocities of the base link expressed in the reference frame of the base.
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  Eigen::VectorXd qjnt, qjntdot, qjntddot;
  
  labrob::ee3 com;
  labrob::ee6 lsole, rsole;
  labrob::ee_rot torso;
  labrob::ee_rot pelvis;

  bool is_left_foot_support, is_right_foot_support;
};

} // end namespace labrob

#endif