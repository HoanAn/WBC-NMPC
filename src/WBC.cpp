JointCommand WalkingManager::compute_inverse_dynamics(
    const GaitConfiguration& current,
    const GaitConfiguration& desired
) {
  // Pinocchio terms
  const auto &J_com = robot_data.Jcom;
  const auto &centroidal_momentum_matrix = pinocchio::ccrba(robot_model, robot_data, current.q, current.qdot);
  const auto &a_com_drift = robot_data.acom[0];
  const auto a_lsole_drift = J_lsole_dot * current.qdot;
  const auto a_rsole_drift = J_rsole_dot * current.qdot;
  const auto a_torso_orientation_drift = J_torso_dot.bottomRows<3>() * current.qdot;
  const auto a_pelvis_orientation_drift = J_pelvis_dot.bottomRows<3>() * current.qdot;

  // Compute desired accelerations
  auto err_com = desired.com.pos - current.com.pos;
  auto err_com_vel = desired.com.vel - current.com.vel;

  auto err_lsole = err_frameplacement(desired.lsole.pos, current.lsole.pos);
  auto err_lsole_vel = desired.lsole.vel - current.lsole.vel;

  auto err_rsole = err_frameplacement(desired.rsole.pos, current.rsole.pos);
  auto err_rsole_vel = desired.rsole.vel - current.rsole.vel;

  auto err_torso_orientation = err_rotation(desired.torso.pos, current.torso.pos);
  auto err_torso_orientation_vel = desired.torso.vel - current.torso.vel;

  auto err_pelvis_orientation = err_rotation(desired.pelvis.pos, current.pelvis.pos);
  auto err_pelvis_orientation_vel = desired.pelvis.vel - current.pelvis.vel;

  Eigen::VectorXd err_posture = (desired.q - current.q).tail(6 + n_joints); // q vector is 6 + n_joints + 1, for some reason
  Eigen::VectorXd err_posture_vel = (desired.qdot - current.qdot).tail(6 + n_joints);
  Eigen::MatrixXd err_posture_selection_matrix = Eigen::MatrixXd::Zero(6 + n_joints, 6 + n_joints);
  err_posture_selection_matrix.block(6, 6, n_joints, n_joints) = Eigen::MatrixXd::Identity(n_joints, n_joints);

  Eigen::MatrixXd cmm_selection_matrix = Eigen::MatrixXd::Zero(3, 6);
  cmm_selection_matrix(0, 3) = cmm_selection_matrix_x;
  cmm_selection_matrix(1, 4) = cmm_selection_matrix_y;
  cmm_selection_matrix(2, 5) = cmm_selection_matrix_z;

  Eigen::VectorXd a_jnt_total = desired.qddot + Kp_regulation * err_posture + Kd_regulation * err_posture_vel;
  Eigen::VectorXd a_com_total = desired.com.acc + Kp_motion * err_com + Kd_motion * err_com_vel;
  Eigen::VectorXd a_lsole_total = desired.lsole.acc + Kp_motion * err_lsole + Kd_motion * err_lsole_vel;
  Eigen::VectorXd a_rsole_total = desired.rsole.acc + Kp_motion * err_rsole + Kd_motion * err_rsole_vel;
  Eigen::VectorXd a_torso_orientation_total = desired.torso.acc + Kp_motion * err_torso_orientation + Kd_motion * err_torso_orientation_vel;
  Eigen::VectorXd a_pelvis_orientation_total = desired.pelvis.acc + Kp_motion * err_pelvis_orientation + Kd_motion * err_pelvis_orientation_vel;

  // Build cost function
  Eigen::MatrixXd H_acc = Eigen::MatrixXd::Zero(6 + n_joints, 6 + n_joints);
  Eigen::VectorXd f_acc = Eigen::VectorXd::Zero(6 + n_joints);

  H_acc += weight_q_ddot * Eigen::MatrixXd::Identity(6 + n_joints, 6 + n_joints);
  H_acc += weight_com * (J_com.transpose() * J_com);
  H_acc += weight_lsole * (J_lsole.transpose() * J_lsole);
  H_acc += weight_rsole * (J_rsole.transpose() * J_rsole);
  H_acc += weight_torso * (J_torso.bottomRows<3>().transpose() * J_torso.bottomRows<3>());
  H_acc += weight_pelvis * (J_pelvis.bottomRows<3>().transpose() * J_pelvis.bottomRows<3>());
  H_acc += weight_regulation * (err_posture_selection_matrix.transpose() * err_posture_selection_matrix);
  H_acc += weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      std::pow(control_timestep, 2.0) * cmm_selection_matrix * centroidal_momentum_matrix;

  f_acc += weight_com * J_com.transpose() * (a_com_drift - a_com_total);
  f_acc += weight_lsole * J_lsole.transpose() * (a_lsole_drift - a_lsole_total);
  f_acc += weight_rsole * J_rsole.transpose() * (a_rsole_drift - a_rsole_total);
  f_acc += weight_torso * J_torso.bottomRows<3>().transpose() * (a_torso_orientation_drift - a_torso_orientation_total);
  f_acc += weight_pelvis * J_pelvis.bottomRows<3>().transpose() * (a_pelvis_orientation_drift - a_pelvis_orientation_total);
  f_acc += -weight_regulation * err_posture_selection_matrix.transpose() * err_posture_selection_matrix * a_jnt_total;
  f_acc += weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      control_timestep * cmm_selection_matrix * centroidal_momentum_matrix * current.qdot;

  auto q_jnt_dot_min = -robot_model.velocityLimit.tail(n_joints);
  auto q_jnt_dot_max = robot_model.velocityLimit.tail(n_joints);
  auto q_jnt_min = robot_model.lowerPositionLimit.tail(n_joints);
  auto q_jnt_max = robot_model.upperPositionLimit.tail(n_joints);

  Eigen::MatrixXd C_acc = Eigen::MatrixXd::Zero(2 * n_joints, 6 + n_joints);
  Eigen::VectorXd d_min_acc(2 * n_joints);
  Eigen::VectorXd d_max_acc(2 * n_joints);
  C_acc.rightCols(n_joints).topRows(n_joints).diagonal().setConstant(control_timestep);
  C_acc.rightCols(n_joints).bottomRows(n_joints).diagonal().setConstant(std::pow(control_timestep, 2.0) / 2.0);
  d_min_acc << q_jnt_dot_min - current.qdot.tail(n_joints), q_jnt_min - current.q.tail(n_joints) - control_timestep * current.qdot.tail(n_joints);
  d_max_acc << q_jnt_dot_max - current.qdot.tail(n_joints), q_jnt_max - current.q.tail(n_joints) - control_timestep * current.qdot.tail(n_joints);

  Eigen::MatrixXd M = pinocchio::crba(robot_model, robot_data, current.q);
  // We need to do this since the inertia matrix in Pinocchio is only upper triangular
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal().tail(n_joints) += M_armature_;

  // Computing Coriolis, centrifugal and gravitational effects
  const auto &c = pinocchio::rnea(robot_model, robot_data, current.q, current.qdot, Eigen::VectorXd::Zero(6 + n_joints));

  Eigen::MatrixXd Jlu = J_lsole.block(0,0,6,6);
  Eigen::MatrixXd Jla = J_lsole.block(0,6,6,n_joints);
  Eigen::MatrixXd Jru = J_rsole.block(0,0,6,6);
  Eigen::MatrixXd Jra = J_rsole.block(0,6,6,n_joints);

  Eigen::MatrixXd Mu = M.block(0,0,6,6+n_joints);
  Eigen::MatrixXd Ma = M.block(6,0,n_joints,6+n_joints);

  Eigen::VectorXd cu = c.block(0,0,6,1);
  Eigen::VectorXd ca = c.block(6,0,n_joints,1);

  std::vector<Eigen::Vector3d> pcis(4);
  pcis[0] << foot_length / 2.0, foot_width / 2.0, 0.0;
  pcis[1] << foot_length / 2.0, -foot_width / 2.0, 0.0;
  pcis[2] << -foot_length / 2.0, foot_width / 2.0, 0.0;
  pcis[3] << -foot_length / 2.0, -foot_width / 2.0, 0.0;

  std::vector<Eigen::Vector3d> pcis_l(4);
  std::vector<Eigen::Vector3d> pcis_r(4);

  for (int i = 0; i < num_contacts_; ++i) {
    pcis_l[i] = T_lsole.rotation() * pcis[i];
    pcis_r[i] = T_rsole.rotation() * pcis[i];
  }

  Eigen::MatrixXd T_l(6, 3 * num_contacts_);
  Eigen::MatrixXd T_r(6, 3 * num_contacts_);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  T_l << I3, I3, I3, I3,
         pinocchio::skew(pcis_l[0]), pinocchio::skew(pcis_l[1]), pinocchio::skew(pcis_l[2]), pinocchio::skew(pcis_l[3]);
  T_r << I3, I3, I3, I3,
         pinocchio::skew(pcis_r[0]), pinocchio::skew(pcis_r[1]), pinocchio::skew(pcis_r[2]), pinocchio::skew(pcis_r[3]);

  Eigen::MatrixXd H_force_one = 1e-9 * Eigen::MatrixXd::Identity(3 * n_contacts, 3 * n_contacts);
  Eigen::VectorXd f_force_one = Eigen::VectorXd::Zero(3 * n_contacts);

  Eigen::VectorXd b_dyn = -cu;

  Eigen::MatrixXd C_force_block(4, 3);
  C_force_block <<  1.0,  0.0, -mu_wbc,
                    0.0,  1.0, -mu_wbc,
                   -1.0,  0.0, -mu_wbc,
                    0.0, -1.0, -mu_wbc;

  Eigen::VectorXd d_min_force_one = -10000.0 * Eigen::VectorXd::Ones(4 * n_contacts);
  Eigen::VectorXd d_max_force_one = Eigen::VectorXd::Zero(4 * n_contacts);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_acc.rows() + 2 * H_force_one.rows(), H_acc.cols() + 2 * H_force_one.cols());
  H.block(0, 0, H_acc.rows(), H_acc.cols()) = H_acc;
  H.block(H_acc.rows(), H_acc.cols(), H_force_one.rows(), H_force_one.cols()) = H_force_one;
  H.block(H_acc.rows() + H_force_one.rows(),
          H_acc.cols() + H_force_one.cols(),
          H_force_one.rows(),
          H_force_one.cols()) = H_force_one;
  Eigen::VectorXd f(f_acc.size() + 2 * f_force_one.size());
  f << f_acc, f_force_one, f_force_one;

  Eigen::MatrixXd A_acc = Eigen::MatrixXd::Zero(12, 6 + n_joints);
  Eigen::VectorXd b_acc = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd A_no_contact = Eigen::MatrixXd::Zero(3 * n_contacts, 2 * 3 * n_contacts);
  Eigen::VectorXd b_no_contact = Eigen::VectorXd::Zero(3 * n_contacts);

  if (current.is_left_foot_support) {
    A_acc.topRows(6) = J_lsole;
    b_acc.topRows(6) = -J_lsole_dot * current.qdot - gamma_wbc * J_lsole * current.qdot;
  }
  if (current.is_right_foot_support) {
    A_acc.bottomRows(6) = J_rsole;
    b_acc.bottomRows(6) = -J_rsole_dot * current.qdot - gamma_wbc * J_rsole * current.qdot;
  }
  if (!current.is_left_foot_support) {
    A_no_contact.block(0, 0, 3 * n_contacts, 3 * n_contacts) = Eigen::MatrixXd::Identity(3 * n_contacts, 3 * n_contacts);
  }
  if (!current.is_right_foot_support) {
    A_no_contact.block(0, 3 * n_contacts, 3 * n_contacts, 3 * n_contacts) = Eigen::MatrixXd::Identity(3 * n_contacts, 3 * n_contacts);
  }

  Eigen::MatrixXd A_dyn(6, 6 + n_joints + 2 * 3 * n_contacts);
  A_dyn << Mu, -Jlu.transpose() * T_l, -Jru.transpose() * T_r;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_acc.rows() + A_no_contact.rows() + A_dyn.rows(), n_wbc_variables);
  A.block(0, 0, A_acc.rows(), A_acc.cols()) = A_acc;
  A.block(A_acc.rows(), A_acc.cols(), A_no_contact.rows(), A_no_contact.cols()) = A_no_contact;
  A.bottomRows(A_dyn.rows()) = A_dyn;
  Eigen::VectorXd b(b_acc.rows() + b_no_contact.rows() + b_dyn.rows());
  b << b_acc, b_no_contact, b_dyn;

  Eigen::MatrixXd C_force_left = Eigen::MatrixXd::Zero(4 * n_contacts, 3 * n_contacts);
  for (int i = 0; i < n_contacts; ++i) {
    C_force_left.block(4 * i, 3 * i, 4, 3) = C_force_block * current.lsole.pos.R.transpose();
  }
  Eigen::MatrixXd C_force_right = Eigen::MatrixXd::Zero(4 * n_contacts, 3 * n_contacts);
  for (int i = 0; i < n_contacts; ++i) {
    C_force_right.block(4 * i, 3 * i, 4, 3) = C_force_block * current.rsole.pos.R.transpose();
  }
  Eigen::MatrixXd C(C_acc.rows() + 2 * C_force_left.rows(), n_wbc_variables);
  C << C_acc, Eigen::MatrixXd::Zero(C_acc.rows(), 2 * 3 * n_contacts),
      Eigen::MatrixXd::Zero(C_force_left.rows(), 6 + n_joints), C_force_left, Eigen::MatrixXd::Zero(C_force_left.rows(), 3 * n_contacts),
      Eigen::MatrixXd::Zero(C_force_right.rows(), 6 + n_joints), Eigen::MatrixXd::Zero(C_force_right.rows(), 3 * n_contacts), C_force_right;
  Eigen::VectorXd d_min(d_min_acc.rows() + 2 * d_min_force_one.rows());
  Eigen::VectorXd d_max(d_max_acc.rows() + 2 * d_max_force_one.rows());
  d_min << d_min_acc, d_min_force_one, d_min_force_one;
  d_max << d_max_acc, d_max_force_one, d_max_force_one;

  wbc_solver_ptr->solve(H, f, A, b, C, d_min, d_max);
  Eigen::VectorXd solution = wbc_solver_ptr->get_solution();
  Eigen::VectorXd q_ddot = solution.head(6 + n_joints);
  Eigen::VectorXd flr = solution.tail(2 * 3 * n_contacts);
  Eigen::VectorXd fl = flr.head(3 * n_contacts);
  Eigen::VectorXd fr = flr.tail(3 * n_contacts);
  Eigen::VectorXd tau = Ma * q_ddot + ca - Jla.transpose() * T_l * fl - Jra.transpose() * T_r * fr;

  JointCommand joint_command;
  for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model.njoints; ++joint_id) {
    const auto &joint_name = robot_model.names[joint_id];
    joint_command[joint_name] = tau[joint_id - 2];
  }
  
  return joint_command;
}