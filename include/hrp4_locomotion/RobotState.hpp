#ifndef LABROB_ROBOT_STATE_H_
#define LABROB_ROBOT_STATE_H_

#include <hrp4_locomotion/JointState.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace labrob {

class RobotState {
 public:
  RobotState();

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Velocities of the base link expressed in the reference frame of the base.
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  labrob::JointState joint_state;

  Eigen::Vector3d total_force;

  std::vector<Eigen::Vector3d> contact_points;
  std::vector<Eigen::Vector3d> contact_forces;
}; // end class RobotState

} // end namespace labrob

#endif // LABROB_ROBOT_STATE_H_