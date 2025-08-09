#ifndef LABROB_UTILS_HPP_
#define LABROB_UTILS_HPP_

// STL
#include <cmath>
#include <deque>

// Eigen
#include <Eigen/Core>

// Pinocchio
#include <pinocchio/multibody/model.hpp>

// hrp4_locomotion
#include <hrp4_locomotion/FootstepPlanElement.hpp>
#include <hrp4_locomotion/LIPState.hpp>
#include <hrp4_locomotion/RobotState.hpp>
#include <hrp4_locomotion/WalkingState.hpp>

namespace labrob {

template <class T>
Eigen::Matrix<T, 3, 3>
Rz(T theta) {
  T c = std::cos(theta);
  T s = std::sin(theta);
  Eigen::Matrix<T, 3, 3> R;
  R <<   c,   -s, 0.0,
        s,    c, 0.0,
      0.0,  0.0, 1.0;
  return R;
}

template <class T>
Eigen::Matrix<T, 2, 2>
Rz_planar(T theta) {
  T c = std::cos(theta);
  T s = std::sin(theta);
  Eigen::Matrix<T, 2, 2> R;
  R << c, -s,
      s,  c;
  return R;
}

template <class T>
T
wrap_angle(T alpha) {
  return std::atan2(std::sin(alpha), std::cos(alpha));
}

template <class T>
T
angle_difference(T alpha, T beta) {
  return wrap_angle(alpha - beta);
}

Eigen::Matrix<double, 6, 1>
err_frameplacement(const pinocchio::SE3& Ta, const pinocchio::SE3& Tb);

Eigen::Vector3d
err_translation(const Eigen::Vector3d& pa, const Eigen::Vector3d& pb);

Eigen::Vector3d
err_rotation(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb);

Eigen::VectorXd
robot_state_to_pinocchio_joint_configuration(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
);

Eigen::VectorXd
robot_state_to_pinocchio_joint_velocity(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
);

Eigen::Vector3d
updateState(
    const LIPState& lip_state,
    double zmpDot,
    int dim,
    double com_target_height,
    int64_t control_timestep_msec
);

labrob::WalkingState
walkingStateFromString(
    const std::string& walking_state_str
);

bool readArgosFootstepPlan(
    const std::string& file_path,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
);

bool readHumanoids2023FootstepPlan(
    const std::string& file_path,
    const labrob::DoubleSupportConfiguration& current_feet_placement,
    const labrob::WalkingState& current_walking_state,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
);

bool readFootstepPlan(
    const std::string& file_path,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
);

void saveFootstepPlan(
    const std::deque<labrob::FootstepPlanElement>& footstep_plan,
    const std::string& file_path
);

void pressAnyKey();

} // end namespace labrob

#endif // LABROB_UTILS_HPP_