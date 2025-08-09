#include <hrp4_locomotion/WBNMPC.hpp>
#include <chrono> 



#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/utils.hpp>

namespace labrob {


WholeBodyMPCParams WholeBodyMPCParams::getDefaultParams() {
  static WholeBodyMPCParams params;

  params.Kp_motion = 30.0;
  params.Kd_motion = 10.0;
  params.Kp_regulation = 30.0;
  params.Kd_regulation = 10.0;

  params.weight_q_ddot = 1e-4;
  params.weight_com = 0.1;
  params.weight_lsole = 1;
  params.weight_rsole = 1;
  params.weight_torso = 1e-3;
  params.weight_pelvis = 0;
  params.weight_angular_momentum = 0.0001;
  params.weight_regulation = 1e-4;

  params.cmm_selection_matrix_x = 1e-6;
  params.cmm_selection_matrix_y = 1e-6;
  params.cmm_selection_matrix_z = 1e-4;

  params.gamma = params.Kd_motion;
  params.mu = 0.5;

  params.foot_length = 0.17;
  params.foot_width = 0.05; 

  return params;
}

WholeBodyMPC::WholeBodyMPC(
    const WholeBodyMPCParams& params, const pinocchio::Model& robot_model,
    const Eigen::VectorXd& q_jnt_reg,
    double sample_time,
    std::map<std::string, double>& armatures)
    : robot_model_(robot_model),
      q_jnt_reg_(q_jnt_reg),
      sample_time_(sample_time),
      params_(params)
{

  robot_data_ = pinocchio::Data(robot_model_);

  lsole_idx_ = robot_model_.getFrameId("left_foot_link");
  rsole_idx_ = robot_model_.getFrameId("right_foot_link");
  torso_idx_ = robot_model_.getFrameId("torso_link");
  pelvis_idx_ = robot_model_.getFrameId("pelvis");

  J_torso_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_pelvis_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_lsole_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_rsole_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  J_torso_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_pelvis_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_lsole_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_rsole_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  n_joints_ = robot_model.nv - 6;
  n_contacts_ = 4;
  n_wbc_variables_ = 6 + n_joints_ + 2 * 3 * n_contacts_;
  n_wbc_equalities_ = 6 + 2 * 6 + 3 * n_contacts_;
  n_wbc_inequalities_ = 2 * n_joints_ + 2 * 4 * n_contacts_;

  M_armature_ = Eigen::VectorXd::Zero(n_joints_);
  for (pinocchio::JointIndex joint_id = 2;
       joint_id < (pinocchio::JointIndex) robot_model_.njoints;
       ++joint_id) {
    std::string joint_name = robot_model_.names[joint_id];
    M_armature_(joint_id - 2) = armatures[joint_name];
  }

  wbc_solver_ptr_ = std::make_unique<qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<qpsolvers::HPIPMQPSolver>(
          n_wbc_variables_, n_wbc_equalities_, n_wbc_inequalities_
      )
  );


  std::cout << "Setting up casadi components" << std::endl; 
  

  SX ca_q = SX::sym("q", robot_model_.nq);
  Ca_ConfigVector cs_q(robot_model_.nq);
  cs_q = Eigen::Map<Ca_ConfigVector>(static_cast<std::vector<SX> > (ca_q).data(), robot_model_.nq,1);
  std::cout << "Done setting cs_q" << std::endl; 

  SX ca_v = SX::sym("v", robot_model_.nv);
  Ca_TangentVector cs_v(robot_model_.nv);
  cs_v = Eigen::Map<Ca_TangentVector>(static_cast<std::vector<SX> > (ca_v).data(), robot_model_.nv,1);
  
  SX ca_pos_v = SX::sym("pos_v", robot_model_.nv);
  
  SX ca_a = (ca_pos_v - ca_v)/ sample_time_;
  Ca_TangentVector cs_a(robot_model_.nv);
  cs_a = Eigen::Map<Ca_TangentVector>(static_cast<std::vector<SX> > (ca_a).data(), robot_model_.nv,1);

  std::cout << "Building casadi robot model" << std::endl;
  CasadiModel model_casadi = robot_model_.cast<SX>();
  CasadiData data_casadi(model_casadi);

  rnea(model_casadi, data_casadi, cs_q, cs_v, cs_a);
  std::cout << "Done building RNEA FUNCTION" << std::endl;
  SX tau_rnea(robot_model_.nv,1);
  for (Eigen::DenseIndex i = 0; i < robot_model_.nv; ++i) {
    tau_rnea(i) = data_casadi.tau[i];
  }
  std::cout << "func RNEA rows, cols: (" << tau_rnea.size1() <<", "<< tau_rnea.size2()<< ")"<< std::endl;
  std::cout << "Building RNEA eval casadi function" << std::endl; 
  // Function eval_rnea("eval_rnea", SXVector{ca_q, ca_v, ca_a}, SXVector{tau_rnea});
  // std::cout << "Done building RNEA eval casadi FUNCTION" << std::endl;

  
  // //SX f_RNEA_ca = pinocchio::rnea(model_casadi, data_casadi, cs_q, cs_v, cs_a);
  std::cout << "Building RNEA Jacobian casadi function" << std::endl; 
  SX input_vars = vertcat(ca_q, ca_v, ca_pos_v);
  SX Jacob_RNEA_ca = jacobian(tau_rnea, input_vars);
  std::cout << "Jacob_RNEA" << tau_rnea << std::endl;
  

  eval_Jacob_rnea_ = Function("eval_Jacob_rnea", SXVector{ca_q, ca_v, ca_pos_v}, SXVector{Jacob_RNEA_ca});

  std::cout << "Done Building RNEA Jacobian eval casadi function" << std::endl; 
  std::cout << "Jacobian RNEA rows, cols: (" << Jacob_RNEA_ca.size1() <<", "<< Jacob_RNEA_ca.size2()<< ")"<< std::endl;
  pressAnyKey();
  std::cout << "Space bar pressed! Program continuing." << std::endl;
  // Dict codegen_options;
  // codegen_options["with_header"] = true;
  // eval_Jacob_rnea_.generate("eval_Jacob_rnea", codegen_options);



}

labrob::JointCommand
WholeBodyMPC::compute_inverse_dynamics(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state,
    pinocchio::Data& robot_data,
    const labrob::GaitConfiguration& current,
    const labrob::GaitConfiguration& desired
) {

  auto start_time = std::chrono::high_resolution_clock::now();

  auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
  auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);

  /// Test the RNEA Jacobian matrix
  std::vector<double> q_vec((size_t)robot_model_.nq);
  Eigen::Map<pinocchio::Model::ConfigVectorType>(q_vec.data(), robot_model_.nq, 1) = q;

  std::vector<double> v_vec((size_t)robot_model_.nv);
  Eigen::Map<pinocchio::Model::ConfigVectorType>(q_vec.data(), robot_model_.nq, 1) = qdot;

  // std::vector<double> q_vec((size_t)robot_model_.nq);
  // Eigen::Map<pinocchio::Model::ConfigVectorType>(q_vec.data(), robot_model_.nq, 1) = q;

  std::cout << "Computing numerical Jacob RNEA" << std::endl;

  DM Jacob_rnea = eval_Jacob_rnea_(DMVector{q_vec, v_vec, v_vec})[0];

  std::cout << "Jacobian RNEA: " << Jacob_rnea << std::endl;
  std::cout << "Jacobian RNEA rows, cols: (" << Jacob_rnea.size1() <<", "<< Jacob_rnea.size2()<< ")"<< std::endl;

  // Compute pinocchio terms
  pinocchio::jacobianCenterOfMass(robot_model, robot_data, q);
  pinocchio::computeJointJacobiansTimeVariation(robot_model, robot_data, q, qdot);
  pinocchio::framesForwardKinematics(robot_model, robot_data, q);

  pinocchio::getFrameJacobian(robot_model, robot_data, torso_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_torso_);
  pinocchio::getFrameJacobian(robot_model, robot_data, pelvis_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_pelvis_);
  pinocchio::getFrameJacobian(robot_model, robot_data, lsole_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_lsole_);
  pinocchio::getFrameJacobian(robot_model, robot_data, rsole_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_rsole_);

  pinocchio::centerOfMass(robot_model, robot_data, q, qdot, 0.0 * qdot); // This is to compute the drift term
  pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, torso_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_torso_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, pelvis_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_pelvis_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, lsole_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_lsole_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, rsole_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_rsole_dot_);

  const auto& J_com = robot_data.Jcom;
  const auto& centroidal_momentum_matrix = pinocchio::ccrba(robot_model, robot_data, q, qdot);
  const auto& a_com_drift = robot_data.acom[0];
  const auto a_lsole_drift = J_lsole_dot_ * qdot;
  const auto a_rsole_drift = J_rsole_dot_ * qdot;
  const auto a_torso_orientation_drift = J_torso_dot_.bottomRows<3>() * qdot;
  const auto a_pelvis_orientation_drift = J_pelvis_dot_.bottomRows<3>() * qdot;

  // Compute desired accelerations
  auto err_com = desired.com.pos - current.com.pos;
  auto err_com_vel = desired.com.vel - current.com.vel;

  auto err_lsole = err_frameplacement(
      pinocchio::SE3(desired.lsole.pos.R, desired.lsole.pos.p),
      pinocchio::SE3(current.lsole.pos.R, current.lsole.pos.p)
  );
  auto err_lsole_vel = desired.lsole.vel - current.lsole.vel;

  auto err_rsole = err_frameplacement(
      pinocchio::SE3(desired.rsole.pos.R, desired.rsole.pos.p),
      pinocchio::SE3(current.rsole.pos.R, current.rsole.pos.p)
  );
  auto err_rsole_vel = desired.rsole.vel - current.rsole.vel;

  auto err_torso_orientation = err_rotation(desired.torso.pos, current.torso.pos);
  auto err_torso_orientation_vel = desired.torso.vel - current.torso.vel;

  auto err_pelvis_orientation = err_rotation(desired.pelvis.pos, current.pelvis.pos);
  auto err_pelvis_orientation_vel = desired.pelvis.vel - current.pelvis.vel;

  Eigen::VectorXd err_posture(6 + n_joints_);
  err_posture << Eigen::VectorXd::Zero(6), desired.qjnt - current.qjnt;
  Eigen::VectorXd err_posture_vel(6 + n_joints_); 
  err_posture_vel << Eigen::VectorXd::Zero(6), desired.qjntdot - current.qjntdot;
  Eigen::MatrixXd err_posture_selection_matrix = Eigen::MatrixXd::Zero(6 + n_joints_, 6 + n_joints_);
  err_posture_selection_matrix.block(6, 6, n_joints_, n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

  Eigen::MatrixXd cmm_selection_matrix = Eigen::MatrixXd::Zero(3, 6);
  cmm_selection_matrix(0, 3) = params_.cmm_selection_matrix_x;
  cmm_selection_matrix(1, 4) = params_.cmm_selection_matrix_y;
  cmm_selection_matrix(2, 5) = params_.cmm_selection_matrix_z;

  Eigen::VectorXd desired_qddot(6 + n_joints_);
  desired_qddot << Eigen::VectorXd::Zero(6), desired.qjntddot;
  Eigen::VectorXd a_jnt_total = desired_qddot + params_.Kp_regulation * err_posture + params_.Kd_regulation * err_posture_vel;
  Eigen::VectorXd a_com_total = desired.com.acc + params_.Kp_motion * err_com + params_.Kd_motion * err_com_vel;
  Eigen::VectorXd a_lsole_total = desired.lsole.acc + params_.Kp_motion * err_lsole + params_.Kd_motion * err_lsole_vel;
  Eigen::VectorXd a_rsole_total = desired.rsole.acc + params_.Kp_motion * err_rsole + params_.Kd_motion * err_rsole_vel;
  Eigen::VectorXd a_torso_orientation_total = desired.torso.acc + params_.Kp_motion * err_torso_orientation + params_.Kd_motion * err_torso_orientation_vel;
  Eigen::VectorXd a_pelvis_orientation_total = desired.pelvis.acc + params_.Kp_motion * err_pelvis_orientation + params_.Kd_motion * err_pelvis_orientation_vel;

  // Build cost function
  Eigen::MatrixXd H_acc = Eigen::MatrixXd::Zero(6 + n_joints_, 6 + n_joints_);
  Eigen::VectorXd f_acc = Eigen::VectorXd::Zero(6 + n_joints_);

  H_acc += params_.weight_q_ddot * Eigen::MatrixXd::Identity(6 + n_joints_, 6 + n_joints_);
  H_acc += params_.weight_com * (J_com.transpose() * J_com);
  H_acc += params_.weight_lsole * (J_lsole_.transpose() * J_lsole_);
  H_acc += params_.weight_rsole * (J_rsole_.transpose() * J_rsole_);
  H_acc += params_.weight_torso * (J_torso_.bottomRows<3>().transpose() * J_torso_.bottomRows<3>());
  H_acc += params_.weight_pelvis * (J_pelvis_.bottomRows<3>().transpose() * J_pelvis_.bottomRows<3>());
  H_acc += params_.weight_regulation * err_posture_selection_matrix;
  H_acc += params_.weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      std::pow(sample_time_, 2.0) * cmm_selection_matrix * centroidal_momentum_matrix;

  f_acc += params_.weight_com * J_com.transpose() * (a_com_drift - a_com_total);
  f_acc += params_.weight_lsole * J_lsole_.transpose() * (a_lsole_drift - a_lsole_total);
  f_acc += params_.weight_rsole * J_rsole_.transpose() * (a_rsole_drift - a_rsole_total);
  f_acc += params_.weight_torso * J_torso_.bottomRows<3>().transpose() * (a_torso_orientation_drift - a_torso_orientation_total);
  f_acc += params_.weight_pelvis * J_pelvis_.bottomRows<3>().transpose() * (a_pelvis_orientation_drift - a_pelvis_orientation_total);
  f_acc += -params_.weight_regulation * err_posture_selection_matrix * a_jnt_total;
  f_acc += params_.weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      sample_time_ * cmm_selection_matrix * centroidal_momentum_matrix * qdot;

  auto q_jnt_dot_min = -robot_model.velocityLimit.tail(n_joints_);
  auto q_jnt_dot_max = robot_model.velocityLimit.tail(n_joints_);
  auto q_jnt_min = robot_model.lowerPositionLimit.tail(n_joints_);
  auto q_jnt_max = robot_model.upperPositionLimit.tail(n_joints_);

  Eigen::MatrixXd C_acc = Eigen::MatrixXd::Zero(2 * n_joints_, 6 + n_joints_);
  Eigen::VectorXd d_min_acc(2 * n_joints_);
  Eigen::VectorXd d_max_acc(2 * n_joints_);
  C_acc.rightCols(n_joints_).topRows(n_joints_).diagonal().setConstant(sample_time_);
  C_acc.rightCols(n_joints_).bottomRows(n_joints_).diagonal().setConstant(std::pow(sample_time_, 2.0) / 2.0);
  d_min_acc << q_jnt_dot_min - current.qjntdot, q_jnt_min - current.qjnt - sample_time_ * current.qjntdot;
  d_max_acc << q_jnt_dot_max - current.qjntdot, q_jnt_max - current.qjnt - sample_time_ * current.qjntdot;

  Eigen::MatrixXd M = pinocchio::crba(robot_model, robot_data, q);
  // We need to do this since the inertia matrix in Pinocchio is only upper triangular
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal().tail(n_joints_) += M_armature_;

  // Computing Coriolis, centrifugal and gravitational effects
  const auto& c = pinocchio::rnea(robot_model, robot_data, q, qdot, Eigen::VectorXd::Zero(6 + n_joints_));

  Eigen::MatrixXd Jlu = J_lsole_.block(0,0,6,6);
  Eigen::MatrixXd Jla = J_lsole_.block(0,6,6,n_joints_);
  Eigen::MatrixXd Jru = J_rsole_.block(0,0,6,6);
  Eigen::MatrixXd Jra = J_rsole_.block(0,6,6,n_joints_);

  Eigen::MatrixXd Mu = M.block(0,0,6,6+n_joints_);
  Eigen::MatrixXd Ma = M.block(6,0,n_joints_,6+n_joints_);

  Eigen::VectorXd cu = c.block(0,0,6,1);
  Eigen::VectorXd ca = c.block(6,0,n_joints_,1);

  std::vector<Eigen::Vector3d> pcis(4);
  pcis[0] <<  params_.foot_length / 2.0,  params_.foot_width / 2.0, 0.0;
  pcis[1] <<  params_.foot_length / 2.0, -params_.foot_width / 2.0, 0.0;
  pcis[2] << -params_.foot_length / 2.0,  params_.foot_width / 2.0, 0.0;
  pcis[3] << -params_.foot_length / 2.0, -params_.foot_width / 2.0, 0.0;

  std::vector<Eigen::Vector3d> pcis_l(4);
  std::vector<Eigen::Vector3d> pcis_r(4);

  for (int i = 0; i < n_contacts_; ++i) {
    pcis_l[i] = desired.lsole.pos.R * pcis[i];
    pcis_r[i] = desired.rsole.pos.R * pcis[i];
  }

  Eigen::MatrixXd T_l(6, 3 * n_contacts_);
  Eigen::MatrixXd T_r(6, 3 * n_contacts_);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  T_l << I3, I3, I3, I3,
         pinocchio::skew(pcis_l[0]), pinocchio::skew(pcis_l[1]), pinocchio::skew(pcis_l[2]), pinocchio::skew(pcis_l[3]);
  T_r << I3, I3, I3, I3,
         pinocchio::skew(pcis_r[0]), pinocchio::skew(pcis_r[1]), pinocchio::skew(pcis_r[2]), pinocchio::skew(pcis_r[3]);

  Eigen::MatrixXd H_force_one = 1e-9 * Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  Eigen::VectorXd f_force_one = Eigen::VectorXd::Zero(3 * n_contacts_);

  Eigen::VectorXd b_dyn = -cu;

  Eigen::MatrixXd C_force_block(4, 3);
  C_force_block <<  1.0,  0.0, -params_.mu,
                    0.0,  1.0, -params_.mu,
                   -1.0,  0.0, -params_.mu,
                    0.0, -1.0, -params_.mu;

  Eigen::VectorXd d_min_force_one = -10000.0 * Eigen::VectorXd::Ones(4 * n_contacts_);
  Eigen::VectorXd d_max_force_one = Eigen::VectorXd::Zero(4 * n_contacts_);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_acc.rows() + 2 * H_force_one.rows(), H_acc.cols() + 2 * H_force_one.cols());
  H.block(0, 0, H_acc.rows(), H_acc.cols()) = H_acc;
  H.block(H_acc.rows(), H_acc.cols(), H_force_one.rows(), H_force_one.cols()) = H_force_one;
  H.block(H_acc.rows() + H_force_one.rows(),
          H_acc.cols() + H_force_one.cols(),
          H_force_one.rows(),
          H_force_one.cols()) = H_force_one;
  Eigen::VectorXd f(f_acc.size() + 2 * f_force_one.size());
  f << f_acc, f_force_one, f_force_one;

  Eigen::MatrixXd A_acc = Eigen::MatrixXd::Zero(12, 6 + n_joints_);
  Eigen::VectorXd b_acc = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd A_no_contact = Eigen::MatrixXd::Zero(3 * n_contacts_, 2 * 3 * n_contacts_);
  Eigen::VectorXd b_no_contact = Eigen::VectorXd::Zero(3 * n_contacts_);

  if (current.is_left_foot_support) {
    A_acc.topRows(6) = J_lsole_;
    b_acc.topRows(6) = -J_lsole_dot_ * qdot - params_.gamma * J_lsole_ * qdot;
  }
  if (current.is_right_foot_support) {
    A_acc.bottomRows(6) = J_rsole_;
    b_acc.bottomRows(6) = -J_rsole_dot_ * qdot - params_.gamma * J_rsole_ * qdot;
  }
  if (!current.is_left_foot_support) {
    A_no_contact.block(0, 0, 3 * n_contacts_, 3 * n_contacts_) = Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  }
  if (!current.is_right_foot_support) {
    A_no_contact.block(0, 3 * n_contacts_, 3 * n_contacts_, 3 * n_contacts_) = Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  }

  Eigen::MatrixXd A_dyn(6, 6 + n_joints_ + 2 * 3 * n_contacts_);
  A_dyn << Mu, -Jlu.transpose() * T_l, -Jru.transpose() * T_r;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_acc.rows() + A_no_contact.rows() + A_dyn.rows(), n_wbc_variables_);
  A.block(0, 0, A_acc.rows(), A_acc.cols()) = A_acc;
  A.block(A_acc.rows(), A_acc.cols(), A_no_contact.rows(), A_no_contact.cols()) = A_no_contact;
  A.bottomRows(A_dyn.rows()) = A_dyn;
  Eigen::VectorXd b(b_acc.rows() + b_no_contact.rows() + b_dyn.rows());
  b << b_acc, b_no_contact, b_dyn;

  Eigen::MatrixXd C_force_left = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_left.block(4 * i, 3 * i, 4, 3) = C_force_block * current.lsole.pos.R.transpose();
  }
  Eigen::MatrixXd C_force_right = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_right.block(4 * i, 3 * i, 4, 3) = C_force_block * current.rsole.pos.R.transpose();
  }
  Eigen::MatrixXd C(C_acc.rows() + 2 * C_force_left.rows(), n_wbc_variables_);
  C << C_acc, Eigen::MatrixXd::Zero(C_acc.rows(), 2 * 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_left.rows(), 6 + n_joints_), C_force_left, Eigen::MatrixXd::Zero(C_force_left.rows(), 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_right.rows(), 6 + n_joints_), Eigen::MatrixXd::Zero(C_force_right.rows(), 3 * n_contacts_), C_force_right;
  Eigen::VectorXd d_min(d_min_acc.rows() + 2 * d_min_force_one.rows());
  Eigen::VectorXd d_max(d_max_acc.rows() + 2 * d_max_force_one.rows());
  d_min << d_min_acc, d_min_force_one, d_min_force_one;
  d_max << d_max_acc, d_max_force_one, d_max_force_one;
  //SOLVE THE WBC 
  wbc_solver_ptr_->solve(H, f, A, b, C, d_min, d_max);
  Eigen::VectorXd solution = wbc_solver_ptr_->get_solution();
  Eigen::VectorXd q_ddot = solution.head(6 + n_joints_);
  Eigen::VectorXd flr = solution.tail(2 * 3 * n_contacts_);
  Eigen::VectorXd fl = flr.head(3 * n_contacts_);
  Eigen::VectorXd fr = flr.tail(3 * n_contacts_);
  Eigen::VectorXd tau = Ma * q_ddot + ca - Jla.transpose() * T_l * fl - Jra.transpose() * T_r * fr;

  // Fine misurazione del tempo
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Stampa del tempo di esecuzione
  std::cout << "Tempo di esecuzione del controllore Whole Body: " << duration << " microsecondi" << std::endl;


  JointCommand joint_command;
  for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model.njoints; ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    joint_command[joint_name] = tau[joint_id - 2];
  }
  
  return joint_command;
}

} // end namespace labrob