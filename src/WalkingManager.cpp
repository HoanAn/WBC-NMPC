#include <hrp4_locomotion/WalkingManager.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>

// Eigen
#include <Eigen/Core>

// Pinocchio
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <hrp4_locomotion/GaitConfiguration.hpp>
#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/TimingLaw.hpp>
#include <hrp4_locomotion/utils.hpp>

namespace labrob {

WalkingManager::WalkingManager() :
    filtered_state_(Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero())
{

}

bool
WalkingManager::init(const labrob::RobotState& initial_robot_state,
                     std::map<std::string, double> &armatures) {
  cov_x = Eigen::Matrix3d::Identity();
  cov_y = Eigen::Matrix3d::Identity();
  cov_z = Eigen::Matrix3d::Identity();

  cov_meas_pos = 1.0e1;
  cov_meas_vel = 1.0e2;
  cov_meas_zmp = 1.0e6;

  cov_mod_pos = 1.0;
  cov_mod_vel = 1.0;
  cov_mod_zmp = 1.0;

  // Read URDF from file:
  std::string robot_description_filename = "../g1_description/unitreeg1.urdf";

  // Build Pinocchio model and data from URDF:
  pinocchio::Model full_robot_model;
  pinocchio::JointModelFreeFlyer root_joint;
  pinocchio::urdf::buildModel(
    robot_description_filename,
    root_joint,
    full_robot_model
  );
  const std::vector<std::string> joint_to_lock_names{};
  std::vector<pinocchio::JointIndex> joint_ids_to_lock;
  for (const auto& joint_name : joint_to_lock_names) {
    if (full_robot_model.existJointName(joint_name)) {
      joint_ids_to_lock.push_back(full_robot_model.getJointId(joint_name));
    }
  }

  robot_model_ = pinocchio::buildReducedModel(
      full_robot_model,
      joint_ids_to_lock,
      pinocchio::neutral(full_robot_model)
  );
  robot_data_ = pinocchio::Data(robot_model_);

  // q_next_prev_ = robot_state_to_pinocchio_joint_configuration(robot_model_, initial_robot_state);
  // v_next_prev_ = robot_state_to_pinocchio_joint_velocity(robot_model_, initial_robot_state);

  // Init desired lsole and rsole poses:
  auto q_init = robot_state_to_pinocchio_joint_configuration(
      robot_model_,
      initial_robot_state
  );
  pinocchio::forwardKinematics(robot_model_, robot_data_, q_init);
  pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q_init);
  pinocchio::framesForwardKinematics(robot_model_, robot_data_, q_init);
  lsole_idx_ = robot_model_.getFrameId("left_foot_link");
  rsole_idx_ = robot_model_.getFrameId("right_foot_link");
  torso_idx_ = robot_model_.getFrameId("torso_link");
  pelvis_idx_ = robot_model_.getFrameId("pelvis");
  const auto& T_lsole_init = robot_data_.oMf[lsole_idx_];
  const auto& T_rsole_init = robot_data_.oMf[rsole_idx_];

  int njnt = robot_model_.nv - 6;

  M_armature_ = Eigen::VectorXd::Zero(njnt);
  for(pinocchio::JointIndex joint_id = 2;
      joint_id < (pinocchio::JointIndex) robot_model_.njoints;
      ++joint_id) {
    std::string joint_name = robot_model_.names[joint_id];
    M_armature_(joint_id - 2) = armatures[joint_name];
  }

  double waist_p_des = 0.0;
  double waist_y_des = 0.0;
  double waist_r_des = 0.0;
  double r_hip_y_des = 0.0;
  double r_hip_r_des = -0.05;
  double r_hip_p_des = -0.44;
  double r_knee_p_des = 0.95;
  double r_ankle_p_des = -0.49;
  double r_ankle_r_des = 0.07;
  double l_hip_y_des = 0.0;
  double l_hip_r_des = -r_hip_r_des;
  double l_hip_p_des = r_hip_p_des;
  double l_knee_p_des = r_knee_p_des;
  double l_ankle_p_des = r_ankle_p_des;
  double l_ankle_r_des = -r_ankle_r_des;
  double r_shoulder_p_des = 0.07;
  double r_shoulder_r_des = -0.14;
  double r_shoulder_y_des = 0.0;
  double r_elbow_p_des = -0.44;
  double l_shoulder_p_des = r_shoulder_p_des;
  double l_shoulder_r_des = -r_shoulder_r_des;
  double l_shoulder_y_des = 0.0;
  double l_elbow_p_des = r_elbow_p_des;

  q_jnt_des_ = q_init.tail(njnt);

  // TODO: init using node handle.
  controller_frequency_ = 600;
  controller_timestep_msec_ = 1000 / controller_frequency_;

  double swing_foot_trajectory_height = 0.1;
  double step_length_x = 0.15;
  double step_length_y = 0.0;
  double step_rotation = 0.0;
  int n_steps = 10;
  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(T_lsole_init.rotation(), T_lsole_init.translation()),
          labrob::SE3(T_rsole_init.rotation(), T_rsole_init.translation()),
          labrob::Foot::RIGHT
      ),
      0.0,
      2000,
      labrob::WalkingState::Init
  ));
  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(T_lsole_init.rotation(), T_lsole_init.translation()),
          labrob::SE3(T_rsole_init.rotation(), T_rsole_init.translation()),
          labrob::Foot::RIGHT
      ),
      0.0,
      5000,
      labrob::WalkingState::Standing
  ));

  double double_support_duration = 600;
  double single_support_duration = 600;
  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(T_lsole_init.rotation(), T_lsole_init.translation()),
          labrob::SE3(T_rsole_init.rotation(), T_rsole_init.translation()),
          labrob::Foot::RIGHT
      ),
      0.0,
      double_support_duration,
      labrob::WalkingState::Starting
  ));

  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(T_lsole_init.rotation(), T_lsole_init.translation()),
          labrob::SE3(T_rsole_init.rotation(), T_rsole_init.translation()),
          labrob::Foot::RIGHT
      ),
      swing_foot_trajectory_height,
      single_support_duration,
      labrob::WalkingState::SingleSupport
  ));
  for (int n = 0; n < n_steps; n += 2) {
    walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz(n * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + n * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::RIGHT
        ),
        0.0,
        double_support_duration,
        labrob::WalkingState::DoubleSupport
    ));
    walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz(n * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + n * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::LEFT
        ),
        swing_foot_trajectory_height,
        single_support_duration,
        labrob::WalkingState::SingleSupport
    ));
    walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz((n + 2) * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + (n + 2) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::LEFT
        ),
        0.0,
        double_support_duration,
        labrob::WalkingState::DoubleSupport
    ));
    walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz((n + 2) * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + (n + 2) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::RIGHT
        ),
        swing_foot_trajectory_height,
        single_support_duration,
        labrob::WalkingState::SingleSupport
    ));
  }
  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::Foot::RIGHT
      ),
      0.0,
      0,
      labrob::WalkingState::Stopping
  ));
  walking_data_.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * T_lsole_init.rotation(), T_lsole_init.translation() + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * T_rsole_init.rotation(), T_rsole_init.translation() + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::Foot::RIGHT
      ),
      0.0,
      2000,
      labrob::WalkingState::Standing
  ));

  // Save and read again footstep plan to double check it's working:
  //std::string footstep_plan_path = "/tmp/ditch-footstep-plan-argos.txt";
  //labrob::saveFootstepPlan(walking_data_.footstep_plan, footstep_plan_path);
  //labrob::readFootstepPlan(footstep_plan_path, walking_data_.footstep_plan);
  //labrob::readArgosFootstepPlan(footstep_plan_path, walking_data_.footstep_plan);

  // Init MPC:
  Eigen::Vector3d p_CoM = robot_data_.com[0];
  int64_t mpc_prediction_horizon_msec = 2000;
  int64_t mpc_timestep_msec = 100;
  double com_target_height = p_CoM.z() - T_lsole_init.translation().z();
  double foot_constraint_square_length = 100; //0.20;
  double foot_constraint_square_width = 100; //0.07;
  Eigen::Vector3d p_ZMP = p_CoM - Eigen::Vector3d(0.0, 0.0, com_target_height);
  filtered_state_ = labrob::LIPState(
      p_CoM,
      Eigen::Vector3d::Zero(),
      p_ZMP
  );
  ismpc_ptr_ = std::make_unique<labrob::ISMPC>(
      mpc_prediction_horizon_msec,
      mpc_timestep_msec,
      std::sqrt(9.81 / com_target_height),
      foot_constraint_square_length,
      foot_constraint_square_width
  );

  auto params = WholeBodyControllerParams::getDefaultParams();
  whole_body_controller_ptr_ = std::make_shared<WholeBodyController>(
      params,
      robot_model_,
      q_jnt_des_,
      0.001 * controller_timestep_msec_,
      armatures
  );

  // Init discrete LIP dynamics:
  discrete_lip_dynamics_ptr_ = std::make_unique<labrob::DiscreteLIPDynamics>(
      std::sqrt(9.81 / com_target_height),
      controller_timestep_msec_
  );

  // Init log files:
  // TODO: may be better to use a proper logging system such as glog.
  mpc_timings_log_file_.open("/tmp/mpc_timings.txt");
  mpc_com_log_file_.open("/tmp/mpc_com.txt");
  mpc_zmp_log_file_.open("/tmp/mpc_zmp.txt");
  //configuration_log_file_.open("/tmp/configuration.txt");
  com_log_file_.open("/tmp/com.txt");
  p_lsole_log_file_.open("/tmp/p_lsole.txt");
  p_rsole_log_file_.open("/tmp/p_rsole.txt");
  v_lsole_log_file_.open("/tmp/v_lsole.txt");
  v_rsole_log_file_.open("/tmp/v_rsole.txt");
  p_lsole_des_log_file_.open("/tmp/p_lsole_des.txt");
  p_rsole_des_log_file_.open("/tmp/p_rsole_des.txt");
  v_lsole_des_log_file_.open("/tmp/v_lsole_des.txt");
  v_rsole_des_log_file_.open("/tmp/v_rsole_des.txt");
  angular_momentum_log_file_.open("/tmp/angular_momentum.txt");
  // fl_log_file_.open("/tmp/fl.txt");
  // fr_log_file_.open("/tmp/fr.txt");
  cop_computed_log_file_.open("/tmp/cop_computed.txt");

  return true;
}

LIPState WalkingManager::updateKF(LIPState filtered, LIPState current, const Eigen::Vector3d &input) {
  double omega = ismpc_ptr_->getOmega();
  double worldTimeStep = 0.001 * static_cast<double>(controller_timestep_msec_);

  double ch = cosh(omega*worldTimeStep);
  double sh = sinh(omega*worldTimeStep);
  Eigen::MatrixXd A_lip = Eigen::MatrixXd::Zero(3,3);
  Eigen::VectorXd B_lip = Eigen::VectorXd::Zero(3);
//  Eigen::VectorXd B_dis = Eigen::VectorXd::Zero(3);
  A_lip << ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
  B_lip << worldTimeStep-sh/omega,1-ch,worldTimeStep;

  Eigen::Vector3d x_measure, y_measure, z_measure;
  if (std::isnan(current.zmp_pos_(0))) {
    x_measure = Eigen::Vector3d(current.com_pos_(0), filtered.com_vel_(0), filtered.zmp_pos_(0));
    y_measure = Eigen::Vector3d(current.com_pos_(1), filtered.com_vel_(1), filtered.zmp_pos_(1));
    z_measure = Eigen::Vector3d(current.com_pos_(2), filtered.com_vel_(2), filtered.zmp_pos_(2));
  } else {
    x_measure = Eigen::Vector3d(current.com_pos_(0), current.com_vel_(0), current.zmp_pos_(0));
    y_measure = Eigen::Vector3d(current.com_pos_(1), current.com_vel_(1), current.zmp_pos_(1));
    z_measure = Eigen::Vector3d(current.com_pos_(2), current.com_vel_(2), current.zmp_pos_(2));
  }
  Eigen::Vector3d x_est = Eigen::Vector3d(filtered.com_pos_(0), filtered.com_vel_(0), filtered.zmp_pos_(0));
  Eigen::Vector3d y_est = Eigen::Vector3d(filtered.com_pos_(1), filtered.com_vel_(1), filtered.zmp_pos_(1));
  Eigen::Vector3d z_est = Eigen::Vector3d(filtered.com_pos_(2), filtered.com_vel_(2), filtered.zmp_pos_(2));

  Eigen::MatrixXd F_kf = A_lip;
  Eigen::MatrixXd G_kf = B_lip;
  Eigen::MatrixXd H_kf = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd R_kf = Eigen::MatrixXd::Identity(3,3);
  R_kf.diagonal() << cov_meas_pos, cov_meas_vel, cov_meas_zmp;
  Eigen::MatrixXd Q_kf = Eigen::MatrixXd::Identity(3,3);
  Q_kf.diagonal() << cov_mod_pos, cov_mod_vel, cov_mod_zmp;

  double input_x = input.x();
  double input_y = input.y();
  double input_z = input.z();

  Eigen::VectorXd x_pred = F_kf * x_est + G_kf * input_x;
  Eigen::MatrixXd cov_x_pred = F_kf * cov_x * F_kf.transpose() + Q_kf;

  Eigen::MatrixXd K_kf = cov_x_pred * H_kf.transpose() * (H_kf * cov_x_pred * H_kf.transpose() + R_kf).inverse();

  x_est = x_pred + K_kf * (x_measure - H_kf * x_pred);
  cov_x = (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf) * cov_x_pred * (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf).transpose() + K_kf * R_kf * K_kf.transpose();

  Eigen::VectorXd y_pred = F_kf * y_est + G_kf * input_y;
  Eigen::MatrixXd cov_y_pred = F_kf * cov_y * F_kf.transpose() + Q_kf;

  K_kf = cov_y_pred * H_kf.transpose() * (H_kf * cov_y_pred * H_kf.transpose() + R_kf).inverse();

  y_est = y_pred + K_kf * (y_measure - H_kf * y_pred);
  cov_y = (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf) * cov_y_pred * (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf).transpose() + K_kf * R_kf * K_kf.transpose();

  Eigen::VectorXd z_pred = F_kf * z_est + G_kf * input_z + Eigen::Vector3d(0.0, -9.81 * worldTimeStep, 0.0);
  Eigen::MatrixXd cov_z_pred = F_kf * cov_z * F_kf.transpose() + Q_kf;

  K_kf = cov_z_pred * H_kf.transpose() * (H_kf * cov_z_pred * H_kf.transpose() + R_kf).inverse();

  z_est = z_pred + K_kf * (z_measure - H_kf * z_pred);
  cov_z = (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf) * cov_z_pred * (Eigen::MatrixXd::Identity(3,3) - K_kf * H_kf).transpose() + K_kf * R_kf * K_kf.transpose();

  current.com_pos_ = Eigen::Vector3d(x_est(0), y_est(0), z_est(0));
  current.com_vel_ = Eigen::Vector3d(x_est(1), y_est(1), z_est(1));
  current.zmp_pos_ = Eigen::Vector3d(x_est(2), y_est(2), z_est(2));

  return current;
}

void
WalkingManager::update(
    const labrob::RobotState& robot_state,
    labrob::JointCommand& joint_command
) {

    auto start_time = std::chrono::system_clock::now();

    double controller_timestep = 0.001 * static_cast<double>(controller_timestep_msec_);

    int njnt = robot_model_.nv - 6; // size of configuration space without floating base

    auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
    auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);

    // // Perform forward kinematics on the whole tree and update robot data:
    // pinocchio::forwardKinematics(robot_model_, robot_data_, q);

    // // NOTE: jacobianCenterOfMass calls forwardKinematics and
    // //       computeJointJacobians.
    // pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q);
    // pinocchio::computeJointJacobiansTimeVariation(robot_model_, robot_data_, q, qdot);
    // pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);
    // pinocchio::centerOfMass(robot_model_, robot_data_, q, qdot, 0.0 * qdot); // This is used to compute the CoM drift (J_com_dot * qdot)
    const auto& centroidal_momentum_matrix = pinocchio::ccrba(
        robot_model_,
        robot_data_,
        q,
        qdot
    );
    auto angular_momentum = (centroidal_momentum_matrix * qdot).tail<3>();

    const auto& p_CoM = robot_data_.com[0];
    const auto& a_CoM_drift = robot_data_.acom[0];
    const auto& J_CoM = robot_data_.Jcom;
    const auto& T_torso = robot_data_.oMf[torso_idx_];
    const auto& T_pelvis = robot_data_.oMf[pelvis_idx_];
    auto torso_orientation = T_torso.rotation();
    auto pelvis_orientation = T_pelvis.rotation();
    Eigen::MatrixXd J_torso = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    pinocchio::getFrameJacobian(
        robot_model_,
        robot_data_,
        torso_idx_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_torso
    );

    // auto J_torso_orientation = J_torso.bottomRows<3>();
    // Eigen::MatrixXd J_torso_dot = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // pinocchio::getFrameJacobianTimeVariation(
    //     robot_model_,
    //     robot_data_,
    //     torso_idx_,
    //     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    //     J_torso_dot
    // );
    // auto J_torso_orientation_dot = J_torso_dot.bottomRows<3>();


    Eigen::MatrixXd J_pelvis = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    pinocchio::getFrameJacobian(
        robot_model_,
        robot_data_,
        pelvis_idx_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_pelvis
    );

    // auto J_pelvis_orientation = J_pelvis.bottomRows<3>();
    // Eigen::MatrixXd J_pelvis_dot = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // pinocchio::getFrameJacobianTimeVariation(
    //     robot_model_,
    //     robot_data_,
    //     pelvis_idx_,
    //     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    //     J_pelvis_dot
    // );
    // auto J_pelvis_orientation_dot = J_pelvis_dot.bottomRows<3>();



    const auto& T_lsole = robot_data_.oMf[lsole_idx_];
    Eigen::MatrixXd J_lsole = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    pinocchio::getFrameJacobian(
        robot_model_,
        robot_data_,
        lsole_idx_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_lsole
    );

    const auto& v_lsole = J_lsole * qdot;
    // Eigen::MatrixXd J_lsole_dot = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // pinocchio::getFrameJacobianTimeVariation(
    //     robot_model_,
    //     robot_data_,
    //     lsole_idx_,
    //     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    //     J_lsole_dot
    //     );
    const auto& T_rsole = robot_data_.oMf[rsole_idx_];
    Eigen::MatrixXd J_rsole = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    pinocchio::getFrameJacobian(
        robot_model_,
        robot_data_,
        rsole_idx_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_rsole
    );
    const auto& v_rsole = J_rsole * qdot;
    // Eigen::MatrixXd J_rsole_dot = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // pinocchio::getFrameJacobianTimeVariation(
    //     robot_model_,
    //     robot_data_,
    //     rsole_idx_,
    //     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    //     J_rsole_dot
    // );

    // Update walking state:
    walking_data_.updateWalkingState(t_msec_);

    // Fill current gait configuration:
    labrob::GaitConfiguration current_gait_configuration;
    current_gait_configuration.qjnt = q.tail(njnt);
    current_gait_configuration.qjntdot = qdot.tail(njnt);

    current_gait_configuration.is_left_foot_support = true;
    current_gait_configuration.is_right_foot_support = true;
    if (walking_data_.getWalkingState() == WalkingState::SingleSupport) {
    if (walking_data_.footstep_plan.front().getFeetPlacement().getSupportFoot() == Foot::LEFT) current_gait_configuration.is_right_foot_support = false;
    else if (walking_data_.footstep_plan.front().getFeetPlacement().getSupportFoot() == Foot::RIGHT) current_gait_configuration.is_left_foot_support = false;
    }

    current_gait_configuration.com.pos = robot_data_.com[0];
    current_gait_configuration.com.vel = robot_data_.vcom[0];

    current_gait_configuration.torso.pos = robot_data_.oMf[torso_idx_].rotation();
    current_gait_configuration.torso.vel = J_torso.bottomRows<3>() * qdot;

    current_gait_configuration.pelvis.pos = robot_data_.oMf[pelvis_idx_].rotation();
    current_gait_configuration.pelvis.vel = J_pelvis.bottomRows<3>() * qdot;

    current_gait_configuration.lsole.pos = labrob::SE3(robot_data_.oMf[lsole_idx_].rotation(), robot_data_.oMf[lsole_idx_].translation());
    current_gait_configuration.lsole.vel = J_lsole * qdot;

    current_gait_configuration.rsole.pos = labrob::SE3(robot_data_.oMf[rsole_idx_].rotation(), robot_data_.oMf[rsole_idx_].translation());
    current_gait_configuration.rsole.vel = J_rsole * qdot;

    double eta2 = std::pow(ismpc_ptr_->getOmega(), 2.0);
    double mass = pinocchio::computeTotalMass(robot_model_);
    // Eigen::Vector3d lip_zmp = p_CoM - robot_state.total_force / (mass * eta2);
    Eigen::Vector3d zmp_3d;
    zmp_3d.z() = robot_state.position(2) - robot_state.total_force.z() / (mass * eta2);
    zmp_3d.x() = 0.0;
    zmp_3d.y() = 0.0;
    for (int i = 0; i < robot_state.contact_points.size(); ++i) {
        auto &pi = robot_state.contact_points[i];
        auto &fi = robot_state.contact_forces[i];
        zmp_3d.x() += (pi.x() * fi.z() / robot_state.total_force.z() + (zmp_3d.z() - pi.z()) * fi.x() / robot_state.total_force.z());
        zmp_3d.y() += (pi.y() * fi.z() / robot_state.total_force.z() + (zmp_3d.z() - pi.z()) * fi.y() / robot_state.total_force.z());
    }

    LIPState measured_state(p_CoM, J_CoM * qdot, zmp_3d);

    filtered_state_ = updateKF(filtered_state_, measured_state, ismpc_ptr_->getInput());
    

    // CoM task:
    auto mpc_t0_ms = std::chrono::system_clock::now();
    ismpc_ptr_->solve(t_msec_, walking_data_, filtered_state_);
    // std::cout << "IS-MPC input: " << ismpc_ptr_->getInput().transpose() << std::endl;
    auto mpc_tf_ms = std::chrono::system_clock::now();
    // const auto& ismpc_optimal_control_input = ismpc_ptr_->getInput();

    // Update the state based on the result of the QP:
    auto lip_state = discrete_lip_dynamics_ptr_->integrate(filtered_state_, ismpc_ptr_->getInput());


    Eigen::Vector3d v_CoM_des = lip_state.com_vel_;
    Eigen::Vector3d p_CoM_des = lip_state.com_pos_;
    Eigen::Vector3d p_ZMP_des = lip_state.zmp_pos_;

    // Fill desired gait configuration:
    labrob::GaitConfiguration desired_gait_configuration;
    desired_gait_configuration.qjnt = q_jnt_des_;
    desired_gait_configuration.qjntdot = Eigen::VectorXd::Zero(njnt);
    desired_gait_configuration.qjntddot = Eigen::VectorXd::Zero(njnt);

    desired_gait_configuration.com.pos = lip_state.com_pos_;
    desired_gait_configuration.com.vel = lip_state.com_vel_;
    desired_gait_configuration.com.acc = eta2 * (lip_state.com_pos_ - lip_state.zmp_pos_) - Eigen::Vector3d(0.0, 0.0, 9.81);

    // Feet tasks
    if (current_gait_configuration.is_left_foot_support && current_gait_configuration.is_right_foot_support) {
        desired_gait_configuration.lsole.pos = walking_data_.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration();
        desired_gait_configuration.lsole.vel = Eigen::VectorXd::Zero(6);
        desired_gait_configuration.lsole.acc = Eigen::VectorXd::Zero(6);
        desired_gait_configuration.rsole.pos = walking_data_.footstep_plan.front().getFeetPlacement().getRightFootConfiguration();
        desired_gait_configuration.rsole.vel = Eigen::VectorXd::Zero(6);
        desired_gait_configuration.rsole.acc = Eigen::VectorXd::Zero(6);
    } else if (walking_data_.footstep_plan.front().getFeetPlacement().getSupportFoot() == Foot::LEFT) {
        desired_gait_configuration.lsole.pos = walking_data_.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration();
        desired_gait_configuration.lsole.vel = Eigen::VectorXd::Zero(6);
        desired_gait_configuration.lsole.acc = Eigen::VectorXd::Zero(6);
        pinocchio::SE3 desired_rsole_pose;
        pinocchio::Motion desired_rsole_vel;
        pinocchio::Motion desired_rsole_acc;
        swingFootTrajectory(desired_rsole_pose, desired_rsole_vel, desired_rsole_acc);
        desired_gait_configuration.rsole.pos.R = desired_rsole_pose.rotation();
        desired_gait_configuration.rsole.pos.p = desired_rsole_pose.translation();
        desired_gait_configuration.rsole.vel << desired_rsole_vel.linear(), desired_rsole_vel.angular();
        desired_gait_configuration.rsole.acc << desired_rsole_acc.linear(), desired_rsole_acc.angular();
    } else {
        desired_gait_configuration.rsole.pos = walking_data_.footstep_plan.front().getFeetPlacement().getRightFootConfiguration();
        desired_gait_configuration.rsole.vel = Eigen::VectorXd::Zero(6);
        desired_gait_configuration.rsole.acc = Eigen::VectorXd::Zero(6);
        pinocchio::SE3 desired_lsole_pose;
        pinocchio::Motion desired_lsole_vel;
        pinocchio::Motion desired_lsole_acc;
        swingFootTrajectory(desired_lsole_pose, desired_lsole_vel, desired_lsole_acc);
        desired_gait_configuration.lsole.pos.R = desired_lsole_pose.rotation();
        desired_gait_configuration.lsole.pos.p = desired_lsole_pose.translation();
        desired_gait_configuration.lsole.vel << desired_lsole_vel.linear(), desired_lsole_vel.angular();
        desired_gait_configuration.lsole.acc << desired_lsole_acc.linear(), desired_lsole_acc.angular();
    }


    // Torso task
    double left_foot_yaw = std::atan2(desired_gait_configuration.lsole.pos.R(1, 0), desired_gait_configuration.lsole.pos.R(0, 0));
    double right_foot_yaw = std::atan2(desired_gait_configuration.rsole.pos.R(1, 0), desired_gait_configuration.rsole.pos.R(0, 0));
    desired_gait_configuration.torso.pos = Rz((left_foot_yaw + right_foot_yaw) / 2.0);
    desired_gait_configuration.torso.vel = (desired_gait_configuration.lsole.vel.tail(3) + desired_gait_configuration.rsole.vel.tail(3)) / 2.0;
    desired_gait_configuration.torso.acc = (desired_gait_configuration.lsole.acc.tail(3) + desired_gait_configuration.rsole.acc.tail(3)) / 2.0;

    // Pelvis task
    desired_gait_configuration.pelvis.pos = Rz((left_foot_yaw + right_foot_yaw) / 2.0);
    desired_gait_configuration.pelvis.vel = (desired_gait_configuration.lsole.vel.tail(3) + desired_gait_configuration.rsole.vel.tail(3)) / 2.0;
    desired_gait_configuration.pelvis.acc = (desired_gait_configuration.lsole.acc.tail(3) + desired_gait_configuration.rsole.acc.tail(3)) / 2.0;


    joint_command = whole_body_controller_ptr_->compute_inverse_dynamics(
        robot_model_,
        robot_state,
        robot_data_,
        current_gait_configuration,
        desired_gait_configuration
    );

    auto end_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "WalkingManager::update() took " << elapsed_time << " us" << std::endl;


    // Update timing in milliseconds.
    // NOTE: assuming update() is actually called every controller_timestep_msec_
    //       milliseconds.
    t_msec_ += controller_timestep_msec_;
    prev_angular_momentum_ = angular_momentum;


    // Log:
    mpc_timings_log_file_ << std::chrono::duration_cast<std::chrono::microseconds>(mpc_tf_ms - mpc_t0_ms).count() << std::endl;
    mpc_com_log_file_ << p_CoM_des.transpose() << std::endl;
    mpc_zmp_log_file_ << p_ZMP_des.transpose() << std::endl;
    com_log_file_ << p_CoM.transpose() << std::endl;
    p_lsole_log_file_ << T_lsole.translation().transpose() << std::endl;
    p_rsole_log_file_ << T_rsole.translation().transpose() << std::endl;
    v_lsole_log_file_ << v_lsole.head<3>().transpose() << std::endl;
    v_rsole_log_file_ << v_rsole.head<3>().transpose() << std::endl;
    p_lsole_des_log_file_ << desired_gait_configuration.lsole.pos.p.transpose() << std::endl;
    p_rsole_des_log_file_ << desired_gait_configuration.rsole.pos.p.transpose() << std::endl;
    v_lsole_des_log_file_ << desired_gait_configuration.lsole.vel.head<3>().transpose() << std::endl;
    v_rsole_des_log_file_ << desired_gait_configuration.rsole.vel.head<3>().transpose() << std::endl;
    angular_momentum_log_file_ << angular_momentum.transpose() << std::endl;
    // fl_log_file_ << output.fl.transpose() << std::endl;
    // fr_log_file_ << output.fr.transpose() << std::endl;
    cop_computed_log_file_ << measured_state.zmp_pos_.transpose() << " " << filtered_state_.zmp_pos_.transpose() << " " << zmp_3d.transpose() << std::endl;

}

int64_t
WalkingManager::get_controller_frequency() const {
  return controller_frequency_;
}

Eigen::MatrixXd
WalkingManager::pseudoinverse(const Eigen::MatrixXd& J, double damp) const {
  auto J_T = J.transpose();
  auto Id = Eigen::MatrixXd::Identity(J.cols(), J.cols());
  return (J_T * J + damp * Id).inverse() * J_T;
}

void
WalkingManager::swingFootTrajectory(
    pinocchio::SE3& swing_foot_pose,
    pinocchio::Motion& swing_foot_velocity,
    pinocchio::Motion& swing_foot_acceleration
) const {
  // NOTE: assuming there are at least two elements in the footstep plan.
  // NOTE: assuming roll and pitch are always zero for the swing foot.
  double t = 0.001 * static_cast<double>(t_msec_ - walking_data_.t0 + controller_timestep_msec_);
  double swing_duration = 0.001 * static_cast<double>(walking_data_.footstep_plan.front().getDuration());
  labrob::QuinticPolynomialTimingLaw timing_law(swing_duration);
  double s = timing_law.eval(t);
  double s_dot = timing_law.eval_dt(t);
  double s_ddot = timing_law.eval_dt_dt(t);

  const auto& feet_placement = walking_data_.footstep_plan[0].getFeetPlacement();
  const auto& target_feet_placement = walking_data_.footstep_plan[1].getFeetPlacement();
  const auto& support_foot_identity = feet_placement.getSupportFoot();
  const auto& support_foot_configuration = feet_placement.getSupportFootConfiguration();
  const auto& starting_swing_foot_configuration = feet_placement.getSwingFootConfiguration();
  const auto& target_swing_foot_configuration =
      (support_foot_identity == labrob::Foot::LEFT ?
             target_feet_placement.getRightFootConfiguration() :
             target_feet_placement.getLeftFootConfiguration()
      );
  const auto& p0 = starting_swing_foot_configuration.p;
  const auto& R0 = starting_swing_foot_configuration.R;
  const auto& pf = target_swing_foot_configuration.p;
  const auto& Rf = target_swing_foot_configuration.R;
  double yaw0 = std::atan2(R0(1, 0), R0(0, 0));
  double yawf = std::atan2(Rf(1, 0), Rf(0, 0));

  pinocchio::SE3 desired_swing_foot_pose;
  desired_swing_foot_pose.translation().x() = p0.x() + (pf.x() - p0.x()) * s;
  desired_swing_foot_pose.translation().y() = p0.y() + (pf.y() - p0.y()) * s;
  double zs = support_foot_configuration.p.z();
  double z0 = p0.z();
  double zf = pf.z();
  double h_z = walking_data_.footstep_plan[0].getSwingFootTrajectoryHeight();
  double a = 2.0 * z0 - 4.0 * h_z + 2.0 * zf - 4.0 * zs;
  double b = 4.0 * h_z - 3.0 * z0 - zf + 4.0 * zs;
  double c = z0;
  desired_swing_foot_pose.translation().z() = a * s * s + b * s + c;
  double desired_swing_foot_yaw = yaw0 + angle_difference(yawf, yaw0) * s;
  desired_swing_foot_pose.rotation() = Rz(desired_swing_foot_yaw);

  pinocchio::Motion desired_swing_foot_velocity(
      Eigen::Vector3d(pf.x() - p0.x(), pf.y() - p0.y(), 2 * a * s + b) * s_dot,
      Eigen::Vector3d(0.0, 0.0, angle_difference(yawf, yaw0)) * s_dot
  );

  pinocchio::Motion desired_swing_foot_acceleration(
      Eigen::Vector3d((pf.x() - p0.x()) * s_ddot, (pf.y() - p0.y()) * s_ddot, 2 * a * s_dot * s_dot + (2 * a * s + b) * s_ddot),
      Eigen::Vector3d(0.0, 0.0, angle_difference(yawf, yaw0)) * s_ddot
  );

  swing_foot_pose = desired_swing_foot_pose;
  swing_foot_velocity = desired_swing_foot_velocity;
  swing_foot_acceleration = desired_swing_foot_acceleration;
}

void
WalkingManager::swingFootTrajectoryBezier(
    pinocchio::SE3& swing_foot_pose,
    pinocchio::Motion& swing_foot_velocity
) const {
  // NOTE: assuming there are at least two elements in the footstep plan.
  // NOTE: assuming roll and pitch are always zero for the swing foot.
  double t = 0.001 * static_cast<double>(t_msec_ - walking_data_.t0 + controller_timestep_msec_);
  double swing_duration = 0.001 * static_cast<double>(walking_data_.footstep_plan.front().getDuration());
  labrob::QuinticPolynomialTimingLaw timing_law(swing_duration);
  double s = timing_law.eval(t);
  double s_dot = timing_law.eval_dt(t);

  const auto& feet_placement = walking_data_.footstep_plan[0].getFeetPlacement();
  const auto& target_feet_placement = walking_data_.footstep_plan[1].getFeetPlacement();
  const auto& support_foot_identity = feet_placement.getSupportFoot();
  const auto& support_foot_configuration = feet_placement.getSupportFootConfiguration();
  const auto& starting_swing_foot_configuration = feet_placement.getSwingFootConfiguration();
  const auto& target_swing_foot_configuration =
      (support_foot_identity == labrob::Foot::LEFT ?
             target_feet_placement.getRightFootConfiguration() :
             target_feet_placement.getLeftFootConfiguration()
      );
  const auto& p0 = starting_swing_foot_configuration.p;
  const auto& R0 = starting_swing_foot_configuration.R;
  const auto& pf = target_swing_foot_configuration.p;
  const auto& Rf = target_swing_foot_configuration.R;
  double yaw0 = std::atan2(R0(1, 0), R0(0, 0));
  double yawf = std::atan2(Rf(1, 0), Rf(0, 0));

  double h_z = walking_data_.footstep_plan[0].getSwingFootTrajectoryHeight();
  double z_max = support_foot_configuration.p.z() + h_z;
  auto p1 = Eigen::Vector3d(p0.x(), p0.y(), z_max);
  auto p2 = Eigen::Vector3d(pf.x(), pf.y(), z_max);

  pinocchio::SE3 desired_swing_foot_pose;
  desired_swing_foot_pose.translation() =
      std::pow(1.0 - s, 3.0) * p0 +
      3.0 * std::pow(1.0 - s, 2.0) * s * p1 +
      3.0 * (1.0 - s) * std::pow(s, 2.0) * p2 +
      std::pow(s, 3.0) * pf;
  double desired_swing_foot_yaw = yaw0 + angle_difference(yawf, yaw0) * s;
  desired_swing_foot_pose.rotation() = Rz(desired_swing_foot_yaw);

  pinocchio::Motion desired_swing_foot_velocity(
      (3.0 * std::pow(1.0 - s, 2.0) * (p1 - p0) + 6.0 * (1.0 - s) * s * (p2 - p1) + 3.0 * std::pow(t, 2.0) * (pf - p2)) * s_dot,
      Eigen::Vector3d(0.0, 0.0, angle_difference(yawf, yaw0)) * s_dot
  );

  swing_foot_pose = desired_swing_foot_pose;
  swing_foot_velocity = desired_swing_foot_velocity;
}

} // end namespace labrob
