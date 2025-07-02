#ifndef LABROB_WALKING_MANAGER_HPP_
#define LABROB_WALKING_MANAGER_HPP_

#include <cmath>
#include <string>
#include <vector>

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hrp4_locomotion/DiscreteLIPDynamics.hpp>
#include <hrp4_locomotion/ISMPC.hpp>
#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/JointState.hpp>
#include <hrp4_locomotion/RobotState.hpp>
#include <hrp4_locomotion/WalkingData.hpp>
#include <hrp4_locomotion/utils.hpp>
#include <hrp4_locomotion/WholeBodyController.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>


namespace labrob {

class WalkingManager {
 public:
  WalkingManager();

  bool init(const labrob::RobotState& initial_robot_state, std::map<std::string, double> &armatures);

  LIPState updateKF(LIPState filtered, LIPState current, const Eigen::Vector3d &input);

  void update(
      const labrob::RobotState& robot_state,
      labrob::JointCommand& joint_command
  );

  int64_t get_controller_frequency() const;

 protected:
  pinocchio::Model robot_model_;
  pinocchio::Data robot_data_;

  pinocchio::FrameIndex lsole_idx_;
  pinocchio::FrameIndex rsole_idx_;
  pinocchio::FrameIndex torso_idx_;
  pinocchio::FrameIndex pelvis_idx_;

  Eigen::VectorXd q_jnt_des_;

  int64_t controller_timestep_msec_;

  labrob::WalkingData walking_data_;
  std::unique_ptr<labrob::ISMPC> ismpc_ptr_;

  Eigen::VectorXd M_armature_;
  Eigen::VectorXd q_next_prev_;
  Eigen::VectorXd v_next_prev_;

  LIPState filtered_state_;
  Eigen::Matrix3d cov_x, cov_y, cov_z;
  double cov_meas_pos, cov_meas_vel, cov_meas_zmp;
  double cov_mod_pos, cov_mod_vel, cov_mod_zmp;

  std::shared_ptr<WholeBodyController> whole_body_controller_ptr_;

private:

  Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& J, double damp=1e-6) const;

  void swingFootTrajectory(
      pinocchio::SE3& swing_foot_pose,
      pinocchio::Motion& swing_foot_velocity,
      pinocchio::Motion& swing_foot_acceleration
  ) const;

  void swingFootTrajectoryBezier(
      pinocchio::SE3& swing_foot_pose,
      pinocchio::Motion& swing_foot_velocity
  ) const;

  int64_t controller_frequency_;
  int64_t t_msec_ = 0;

  std::unique_ptr<labrob::DiscreteLIPDynamics> discrete_lip_dynamics_ptr_;

  Eigen::Vector3d prev_angular_momentum_ = Eigen::Vector3d::Zero();

  // Log files:
  std::ofstream mpc_timings_log_file_;
  std::ofstream mpc_com_log_file_;
  std::ofstream mpc_zmp_log_file_;
  //std::ofstream configuration_log_file_;
  std::ofstream com_log_file_;
  std::ofstream p_lsole_log_file_;
  std::ofstream p_rsole_log_file_;
  std::ofstream v_lsole_log_file_;
  std::ofstream v_rsole_log_file_;
  std::ofstream p_lsole_des_log_file_;
  std::ofstream p_rsole_des_log_file_;
  std::ofstream v_lsole_des_log_file_;
  std::ofstream v_rsole_des_log_file_;
  std::ofstream angular_momentum_log_file_;
  //std::ofstream fl_log_file_;
  //std::ofstream fr_log_file_;
  std::ofstream cop_computed_log_file_;

}; // end class WalkingManager

} // end namespace labrob

#endif // LABROB_WALKING_MANAGER_HPP_