#include <fstream>

#include <hrp4_locomotion/WBNMPC.hpp>
#include <chrono> 



#include <hrp4_locomotion/JointCommand.hpp>
//#include <hrp4_locomotion/utils.hpp>


namespace labrob {


WholeBodyMPCParams WholeBodyMPCParams::getDefaultParams() {
  static WholeBodyMPCParams params;

  params.Kp_motion = 30.0;
  params.Kd_motion = 10.0;
  params.Kp_regulation = 30.0;
  params.Kd_regulation = 10.0;

  params.weight_q_ddot = 1e-4;

  params.weight_lsole = 1;
  params.weight_rsole = 1;
  
  params.weight_pelvis = 0;
  params.weight_regulation = 1e-4;
  params.weight_angular_momentum = 0.0001;

  /// Weight needed
  params.weight_com_xy = 10; // First 2 components of vector q
  params.weight_com_z = 100; // 
  params.weight_torso = 1000; // The next fourth quaternion component of vector q
  params.weight_general_qj = 1; // The rest of joint position

  params.weight_general_vb = 0.1; // The first three components of vector v (linear velocity of the base)
  params.weight_general_omega_b = 0.1; // The next three components of vector v (angular velocity of the base)
  params.weight_general_v = 0.01; // The rest of joint velocity

  params.weight_contact_force_xy = 0.1; // Contact forces in x and y direction
  params.weight_contact_force_z = 0.1; // Contact forces in z direction

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
    const Eigen::VectorXd& q_init,
    double dynamic_discretization_time,
    int N,
    std::map<std::string, double>& armatures)
    : robot_model_(robot_model),
      q_jnt_reg_(q_init.tail(robot_model.nv - 6)),
      dynamic_discretization_time_(dynamic_discretization_time),
      N_(N),
      params_(params)
{

  robot_data_ = pinocchio::Data(robot_model_);


  lsole_idx_ = robot_model_.getFrameId("left_foot_link");
  rsole_idx_ = robot_model_.getFrameId("right_foot_link");
  torso_idx_ = robot_model_.getFrameId("torso_link");
  //pelvis_idx_ = robot_model_.getFrameId("pelvis");

  J_torso_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_pelvis_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_lsole_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_rsole_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  

  n_joints_ = robot_model.nv - 6;

  int num_q = robot_model_.nq*(N+1*0); // including q0
  int num_v = robot_model_.nv*(N+1*0); // including v0
  int num_contact = 4; // number of contacts per foot (Heel (l,r) and Toe (l,r))
  int num_force = num_contact*3* N; // consider 1 foot, 3 forces per contact (Fx, Fy, Fz), N time steps
  int num_torques_single_step = robot_model_.nq-7; // excluding q0, q1, q2, q3, q4, q5, q6 (base link)
  int num_torques = num_torques_single_step*(N); // joint torques


  int num_q_single_step = robot_model_.nq; // = 28
  int num_v_single_step = robot_model_.nv; // = 27


  int num_force_single_foot_single_step = num_contact * 3; // 3 forces (Fx, Fy, Fz) per contact

  int num_constraint = 0*num_q_single_step+ 0*num_v_single_step+ N* ( num_q_single_step + // f_kin
                                                                  num_v_single_step + // f_dyn
                                                                  num_force_single_foot_single_step*2 + // f_Fswing
                                                                  2*num_contact + // f_friction
                                                                  2*2 + // f_feet_height
                                                                  num_contact*2*2); // f_tang_contact_vel

  n_wbnmpc_variables_ = num_q + num_v + num_torques + 2 * num_force; // ca_q, ca_v,  tau, F_lsole, F_rsole
  n_wbnmpc_equalities_ = num_constraint - N * 2 * num_contact; // excluding the friction
  n_wbnmpc_inequalities_ = N * 2 * num_contact;
  n_contacts_ = num_contact;




  M_armature_ = Eigen::VectorXd::Zero(n_joints_);
  for (pinocchio::JointIndex joint_id = 2;
       joint_id < (pinocchio::JointIndex) robot_model_.njoints;
       ++joint_id) {
    std::string joint_name = robot_model_.names[joint_id];
    M_armature_(joint_id - 2) = armatures[joint_name];
  }


  // Intialize the size  for the guess vars

  guess_vars_.q_sol.resize((size_t)num_q + robot_model.nq);
  guess_vars_.v_sol.resize((size_t)num_v + robot_model.nv);
  guess_vars_.F_lsole_sol.resize((size_t)num_force);
  guess_vars_.F_rsole_sol.resize((size_t)num_force);
  guess_vars_.tau_sol.resize((size_t)num_torques);

  update_guess(robot_model_, robot_data_, q_init, Eigen::VectorXd::Zero(robot_model_.nv), N_, true);
  desired_vars_ = guess_vars_;
  std::cout << "Desired vars q:" << desired_vars_.q_sol.transpose() << std::endl;
  solution_vars_ = guess_vars_;

  //wbnmpc_solver_ptr_->P_ =
  g_.resize(n_wbnmpc_variables_);
  g_.setZero();


  const casadi_real* data_cost[10];
  data_cost[0] = desired_vars_.q_sol.data();
  data_cost[1] = &params.weight_com_xy;
  data_cost[2] = &params.weight_com_z;
  data_cost[3] = &params.weight_torso;
  data_cost[4] = &params.weight_general_qj;
  data_cost[5] = &params.weight_general_vb;
  data_cost[6] = &params.weight_general_omega_b;
  data_cost[7] = &params.weight_general_v;
  data_cost[8] = &params.weight_contact_force_xy;
  data_cost[9] = &params.weight_contact_force_z;
  eval_codegen(eval_Hessian_cost_work, eval_Hessian_cost,eval_Hessian_cost_sparsity_out,data_cost, P_ );
  std::cout << "Weight matrix P in CSC format eval: \n" << std::endl;
  for (long long i = 0; i < P_.nzeros; i++){
      std::cout << P_.data[i] << ", ";
  };
  std::cout << std::endl;
  
  qpsolvers::CSCMatrix_params P_upper;
    P_upper.nrows = P_.nrows;
    P_upper.ncols = P_.ncols;
    P_upper.col_pointers.push_back(0);

    for (int j = 0; j < P_.nrows; ++j) {
        int col_start = P_.col_pointers[j];
        int col_end   = P_.col_pointers[j+1];

        for (int k = col_start; k < col_end; ++k) {
            if (P_.row_indices[k] <= j) {          // keep only upper-triangular
                P_upper.row_indices.push_back(P_.row_indices[k]);
                P_upper.data.push_back(P_.data[k]);
            }
        }
        P_upper.col_pointers.push_back(P_upper.row_indices.size());
    }
    P_upper.nzeros = P_upper.row_indices.size();


  std::cout << "P_ nrows: " << P_.nrows << ", ncols: " << P_.ncols << ", nzeros: " << P_.nzeros << std::endl;
  std::cout << "P_upper_ nrows: " << P_upper.nrows << ", ncols: " << P_upper.ncols << ", nzeros: " << P_upper.nzeros << std::endl;
  std::cout << "Weight matrix P_upper in CSC format eval: \n" << std::endl;
  for (long long i = 0; i < P_upper.nzeros; i++){
      std::cout << P_upper.data[i] << ", ";
  };
  std::cout << std::endl;
  
  std::cout << "Init OSQP solver" << std::endl;
  wbnmpc_solver_ptr_ = std::make_unique<qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<qpsolvers::OSQPSolver>(
          n_wbnmpc_variables_, n_wbnmpc_equalities_, n_wbnmpc_inequalities_,P_upper
      )
  );
  pressAnyKey();
  std::cout << "Space bar pressed! Program continuing." << std::endl;
}


void WholeBodyMPC::update_guess(const pinocchio::Model& robot_model,
                                            pinocchio::Data& robot_data,
                                      const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& qdot, 
                                      const int prediction_horizon, const bool init)
{
  static std::ofstream Guess_print ("Guess.txt");
  
  
  // std::cout << "Initial tau after considering contact forces: \n" << tau_init.transpose() << std::endl;
  // std::cout << "Initial tau from RNEA without contact forces test: \n" << tau_init_test.transpose() << std::endl;
  // std::cout << "Initial tau from RNEA with contact forces: \n" << tau_rnea.transpose() << std::endl;
  // std::cout << "Init " << init << std::endl;
  if (init){
    double total_mass = 0.0;
    for (const auto &inertial : robot_model.inertias) {
        total_mass += inertial.mass();
    }
    pinocchio::FrameIndex lsole_idx = robot_model.getFrameId("left_foot_link");
    pinocchio::FrameIndex rsole_idx = robot_model.getFrameId("right_foot_link");
    pinocchio::FrameIndex torso_idx = robot_model.getFrameId("torso_link");
    pinocchio::FrameIndex pelvis_idx = robot_model.getFrameId("pelvis");
    auto lsole_frame = robot_model.frames[lsole_idx];
    auto rsole_frame = robot_model.frames[rsole_idx];
    
    Eigen::MatrixXd J_lsole = Eigen::MatrixXd::Zero(6, robot_model.nv);
    Eigen::MatrixXd J_rsole = Eigen::MatrixXd::Zero(6, robot_model.nv);
    pinocchio::forwardKinematics(robot_model_, robot_data_, q);
    //pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q);
    pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);
    pinocchio::computeJointJacobians(robot_model_, robot_data_, q);
  // std::cout << "lsole frame position" << robot_data.oMf[lsole_idx] << std::endl;
  // std::cout << "torso frame position" << robot_data.oMf[torso_idx] << std::endl;
  // std::cout << "pelvis frame position" << robot_data.oMf[pelvis_idx] << std::endl;
    pinocchio::getFrameJacobian(
        robot_model,
        robot_data,
        lsole_idx,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_lsole
    );
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "J_lsole:\n" << J_lsole.transpose().format(CleanFmt) << std::endl;
    pinocchio::getFrameJacobian(
        robot_model,
        robot_data,
        rsole_idx,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_rsole
    );
  
  //std::cout << "J_rsole:\n" << J_rsole.transpose().format(CleanFmt) << std::endl;

 

    Eigen::VectorXd qdot_init = Eigen::VectorXd::Zero(robot_model.nv);
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(robot_model.nv);

    Eigen::VectorXd F_lsole_init = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd F_rsole_init = Eigen::VectorXd::Zero(12);
    F_lsole_init(2) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_lsole_init(5) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_lsole_init(8) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_lsole_init(11) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot

    F_rsole_init(2) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_rsole_init(5) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_rsole_init(8) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot
    F_rsole_init(11) = total_mass*1 * 9.81 / 8.0; // Quarter of the weight on each contact point/foot

    pinocchio::rnea(robot_model, robot_data, q, qdot, qddot);

    Eigen::VectorXd tau_init = Eigen::VectorXd::Zero(robot_model.nv);
    Eigen::Map<pinocchio::Model::TangentVectorType>(tau_init.data(), robot_model.nv, 1) = robot_data.tau;
    std::cout << "Initial tau from RNEA: \n" << tau_init.transpose() << std::endl;
    Eigen::VectorXd tau_base = tau_init.head(6); 

  //Eigen::VectorXd

    Eigen::Vector3d F_lsole_heel_l_init = F_lsole_init.head(3);
    Eigen::Vector3d F_lsole_heel_r_init = F_lsole_init.segment(3,3);
    Eigen::Vector3d F_lsole_toe_l_init = F_lsole_init.segment(6,3);
    Eigen::Vector3d F_lsole_toe_r_init = F_lsole_init.tail(3);

    Eigen::Vector3d F_rsole_heel_l_init = F_rsole_init.head(3);
    Eigen::Vector3d F_rsole_heel_r_init = F_rsole_init.segment(3,3);
    Eigen::Vector3d F_rsole_toe_l_init = F_rsole_init.segment(6,3);
    Eigen::Vector3d F_rsole_toe_r_init = F_rsole_init.tail(3);

    Eigen::Vector3d heel_l_pos_local = Eigen::Vector3d(-params_.foot_length/2, params_.foot_width/2, 0.0); // Heel_left position in the local frame
    Eigen::Vector3d heel_r_pos_local = Eigen::Vector3d(-params_.foot_length/2, -params_.foot_width/2, 0.0); // Heel_right position in the local frame
  
    Eigen::Vector3d toe_l_pos_local = Eigen::Vector3d(params_.foot_length/2, params_.foot_width/2, 0.0); // Toe_left position in the local frame
    Eigen::Vector3d toe_r_pos_local = Eigen::Vector3d(params_.foot_length/2, -params_.foot_width/2, 0.0); // Toe_right position in the local frame
  
    Eigen::Matrix3d R_world_lfoot = robot_data.oMf[lsole_idx].rotation();
    Eigen::Matrix3d R_world_rfoot = robot_data.oMf[rsole_idx].rotation();

    Eigen::Vector3d heel_l_pos_world_L = R_world_lfoot * heel_l_pos_local;
    Eigen::Vector3d heel_r_pos_world_L = R_world_lfoot * heel_r_pos_local;
    Eigen::Vector3d toe_l_pos_world_L  = R_world_lfoot * toe_l_pos_local;
    Eigen::Vector3d toe_r_pos_world_L  = R_world_lfoot * toe_r_pos_local;
        
    Eigen::Vector3d heel_l_pos_world_R = R_world_rfoot * heel_l_pos_local;
    Eigen::Vector3d heel_r_pos_world_R = R_world_rfoot * heel_r_pos_local;
    Eigen::Vector3d toe_l_pos_world_R  = R_world_rfoot * toe_l_pos_local;
    Eigen::Vector3d toe_r_pos_world_R  = R_world_rfoot * toe_r_pos_local;




    Eigen::VectorXd wrench_lsole_init(6);
    Eigen::VectorXd wrench_rsole_init(6);
    wrench_lsole_init.head(3) = F_lsole_heel_l_init + F_lsole_heel_r_init + F_lsole_toe_l_init + F_lsole_toe_r_init;
    wrench_lsole_init.tail(3) =   heel_l_pos_world_L.cross(F_lsole_heel_l_init) + heel_r_pos_world_L.cross(F_lsole_heel_r_init) 
                              + toe_l_pos_world_L.cross(F_lsole_toe_l_init) + toe_r_pos_world_L.cross(F_lsole_toe_r_init);

    wrench_rsole_init.head(3) = F_rsole_heel_l_init + F_rsole_heel_r_init + F_rsole_toe_l_init + F_rsole_toe_r_init;
    wrench_rsole_init.tail(3) = heel_l_pos_world_R.cross(F_rsole_heel_l_init) + heel_r_pos_world_R.cross(F_rsole_heel_r_init) 
                              + toe_l_pos_world_R.cross(F_rsole_toe_l_init) + toe_r_pos_world_R.cross(F_rsole_toe_r_init);

  std::cout << "Initial wrench left sole: \n" << wrench_lsole_init.transpose() << std::endl;
  std::cout << "Initial wrench right sole: \n" << wrench_rsole_init.transpose() << std::endl;

  // std::cout << "Initial wrench left sole in world frame: \n" << wrench_l.toVector().transpose() << std::endl;
  // std::cout << "Initial wrench right sole in world frame: \n" << wrench_r.toVector().transpose() << std::endl;

  //Eigen::VectorXd tau_init_test = Eigen::VectorXd::Zero(robot_model.nv);
  //tau_init_test = tau_init - J_lsole.transpose() * T_l * F_lsole_init - J_rsole.transpose() * T_r * F_rsole_init;
    tau_init = tau_init - J_lsole.transpose() * wrench_lsole_init - J_rsole.transpose() * wrench_rsole_init;
    std::cout << "Initial tau after considering contact forces: \n" << tau_init.transpose() << std::endl;

    guess_vars_.q_sol = q.replicate(prediction_horizon+1,1);
    guess_vars_.v_sol = qdot.replicate(prediction_horizon+1,1);
    guess_vars_.F_lsole_sol = F_lsole_init.replicate(prediction_horizon,1);
    guess_vars_.F_rsole_sol = F_rsole_init.replicate(prediction_horizon,1);
    guess_vars_.tau_sol = tau_init.tail(robot_model.nv-6).replicate(prediction_horizon,1);
  };  
  
  
  if(!init){


    //guess_vars_.q_sol = q.replicate(prediction_horizon+1,1);
    //guess_vars_.v_sol = qdot.replicate(prediction_horizon+1,1);
    //guess_vars_.F_lsole_sol = F_lsole_init.replicate(prediction_horizon,1);
    //guess_vars_.F_rsole_sol = F_rsole_init.replicate(prediction_horizon,1);
    //guess_vars_.tau_sol = tau_init.tail(robot_model.nv-6).replicate(prediction_horizon,1);



    //guess_vars_ = solution_vars_; // Warm start with the previous solution
    guess_vars_.q_sol.head(robot_model.nq) = q;
    guess_vars_.v_sol.head(robot_model.nv) = qdot;
  
    //Shifted the previous solution to the left
    // guess_vars_.q_sol.segment(robot_model.nq, (N_-1)*robot_model.nq) = solution_vars_.q_sol.segment(2*robot_model.nq, (N_-1)*robot_model.nq);
    // guess_vars_.q_sol.tail(robot_model.nq) = solution_vars_.q_sol.tail(robot_model.nq);
    // for (int i=0; i<N_+1; i++){
    //     // Normalize the quaternion part
    //     guess_vars_.q_sol.segment(i*robot_model.nq +3,4) /= guess_vars_.q_sol.segment(i*robot_model.nq +3,4).norm();
        
    // }
    // guess_vars_.v_sol.segment(robot_model.nv, (N_-1)*robot_model.nv) = solution_vars_.v_sol.segment(2*robot_model.nv, (N_-1)*robot_model.nv);
    // guess_vars_.v_sol.tail(robot_model.nv) = solution_vars_.v_sol.tail(robot_model.nv);

    // guess_vars_.F_lsole_sol.segment(0,(N_-1)*12) = solution_vars_.F_lsole_sol.segment(12,(N_-1)*12);
    // guess_vars_.F_lsole_sol.tail(12) = solution_vars_.F_lsole_sol.tail(12);
    // guess_vars_.F_rsole_sol.segment(0,(N_-1)*12) = solution_vars_.F_rsole_sol.segment(12,(N_-1)*12);
    // guess_vars_.F_rsole_sol.tail(12) = solution_vars_.F_rsole_sol.tail(12);

    // guess_vars_.tau_sol.segment(0,(N_-1)*(robot_model.nv-6)) = solution_vars_.tau_sol.segment(robot_model.nv-6,(N_-1)*(robot_model.nv-6));
    // guess_vars_.tau_sol.tail(robot_model.nv-6) = solution_vars_.tau_sol.tail(robot_model.nv-6);
    
    


    



    //Recompute the RNEA for the first tau
    // Eigen::VectorXd qddot_aprox = (guess_vars_.v_sol.segment(robot_model.nv,robot_model.nv) - guess_vars_.v_sol.head(robot_model.nv)) / 0.01;

    // pinocchio::rnea(robot_model, robot_data, q, qdot, qddot_aprox);
    
    // Eigen::Map<pinocchio::Model::TangentVectorType>(tau_init.data(), robot_model.nv, 1) = robot_data.tau;
    // std::cout << "Updated tau from RNEA in the loop: \n" << tau_init.transpose() << std::endl;
    // std::cout << "Force left in loop: " << guess_vars_.F_lsole_sol.head(6).transpose() << std::endl;
    // std::cout << "Force right in loop: " << guess_vars_.F_rsole_sol.head(6).transpose() << std::endl;
    // std::cout << "Jacobian lsole: \n" << J_lsole.transpose().format(CleanFmt) << std::endl;
    // std::cout << "Jacobian rsole: \n" << J_rsole.transpose().format(CleanFmt) << std::endl;
    // std::cout << "External force contribute: " << (- J_lsole.transpose()* T_l * guess_vars_.F_lsole_sol.head(6) - J_rsole.transpose()* T_r * guess_vars_.F_rsole_sol.head(6)).transpose() << std::endl;
    // tau_init = tau_init - J_lsole.transpose()* T_l * guess_vars_.F_lsole_sol.head(6) - J_rsole.transpose()* T_r * guess_vars_.F_rsole_sol.head(6);
    // std::cout << "tau init guess in loop: " << tau_init.transpose() << std::endl;
    // guess_vars_.tau_sol.head(robot_model_.nv -6 ) = tau_init.tail(robot_model.nv-6);

    // std::cout << " tau from last solution: \n" << guess_vars_.tau_sol.transpose() << std::endl; 
    //guess_vars_.tau_sol.head(robot_model.nv-6) = tau_init.tail(robot_model.nv-6);
    // std::cout << "F from last solution: \n" << guess_vars_.F_lsole_sol.transpose() << std::endl;
    //std::cout << "tau after update with RNEA: \n" << guess_vars_.tau_sol.transpose() << std::endl;
    // std::cout << "Update the torque guess also" << std::endl;
    // guess_vars_.F_lsole_sol = F_lsole_init.replicate(prediction_horizon,1);
    // guess_vars_.F_rsole_sol = F_rsole_init.replicate(prediction_horizon,1);
  
  }

  Guess_print << "q last solution:\n" << solution_vars_.q_sol.transpose() << std::endl;
  Guess_print << "v last solution:\n" << solution_vars_.v_sol.transpose() << std::endl;
  Guess_print << "F_lsole last solution:\n" << solution_vars_.F_lsole_sol.transpose() << std::endl;
  Guess_print << "F_rsole last solution:\n" << solution_vars_.F_rsole_sol.transpose() << std::endl;
  Guess_print << "tau last solution:\n" << solution_vars_.tau_sol.transpose() << std::endl;

  Guess_print << "q guess:\n" << guess_vars_.q_sol.transpose() << std::endl;
  Guess_print << "v guess:\n" << guess_vars_.v_sol.transpose() << std::endl;
  Guess_print << "F_lsole guess:\n" << guess_vars_.F_lsole_sol.transpose() << std::endl;
  Guess_print << "F_rsole guess:\n" << guess_vars_.F_rsole_sol.transpose() << std::endl;
  Guess_print << "tau guess:\n" << guess_vars_.tau_sol.transpose() << std::endl;

  Guess_print << "----------------------------------------" << std::endl;
  // std:: cout << "Updated first guess q_sol: \n" << guess_vars_.q_sol.transpose() << std::endl;
  // std:: cout << "Updated first guess v_sol: \n" << guess_vars_.v_sol.transpose() << std::endl;
  // std:: cout << "Updated first guess F_lsole_sol: \n" << guess_vars_.F_lsole_sol.transpose() << std::endl;
  // std:: cout << "Updated first guess F_rsole_sol: \n" << guess_vars_.F_rsole_sol.transpose() << std::endl;
  // std:: cout << "Updated first guess tau_sol: \n" << guess_vars_.tau_sol.transpose() << std::endl;

  Gamma_vec_ = std::vector<double>(prediction_horizon * 4,0.0); // All contacts are active in the first guess
  
  // desired_vars_ = guess_vars_;
  // solution_vars_ = guess_vars_;

  

  //pressAnyKey();

};

labrob::JointCommand
WholeBodyMPC::compute_inverse_dynamics(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state,
    pinocchio::Data& robot_data,
    const labrob::GaitConfiguration& current,
    const labrob::GaitConfiguration& desired,
    int64_t t_msec,
    const labrob::WalkingData& walking_data
) {

  auto start_time = std::chrono::high_resolution_clock::now();
  double dt = 0.02;

   auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
   //std::cout << "Current q: " << q.transpose() << std::endl;
   
   auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);
   //std::cout << "Current v: " << qdot.transpose() << std::endl;
  // Update for the q0 and v0
  //  guess_vars_.q_sol.head(robot_model.nq) = q;
  //  guess_vars_.v_sol.head(robot_model.nv) = qdot;
  if(t_msec % 5 == 0){
  if (t_msec != 0)
    update_guess(robot_model, robot_data, q, qdot, N_, false);
  
  if(t_msec == 5){
    launch_plot_script();
  }
  static std::ofstream State_print ("State.txt");

  
  // std::cout << "q0 MPC: " << solution_vars_.q_sol.head(robot_model_.nq).transpose() << std::endl;
  // std::cout << "q0 MPC guess, i.e curent meas: " << guess_vars_.q_sol.head(robot_model_.nq).transpose() << std::endl;
  // std::cout << "Predicted q MPC: " << solution_vars_.q_sol.segment(robot_model_.nq,robot_model_.nq).transpose() << std::endl;
  // std::cout << "--------------------------------" << std::endl;
  // std::cout << "v0 MPC: " << solution_vars_.v_sol.head(robot_model_.nv).transpose() << std::endl;
  // std::cout << "v0 MPC guess, i.e curent meas: " << guess_vars_.v_sol.head(robot_model_.nv).transpose() << std::endl;
  // std::cout << "Predicted v MPC: " << solution_vars_.v_sol.segment(robot_model_.nv,robot_model_.nv).transpose() << std::endl;

  State_print << "State at time " << t_msec << " ms ------------------\n";
  State_print << "q measurement:\n" << guess_vars_.q_sol.head(robot_model_.nq).transpose() << std::endl;
  State_print << "q0 MPC last solution:\n" << solution_vars_.q_sol.head(robot_model_.nq).transpose() << std::endl;
  State_print << "Predicted q MPC over horizon:\n" << solution_vars_.q_sol.transpose() << std::endl;
  State_print << "Quaternion norm:\n";
  for (int i = 1; i< N_ ; i++){
    State_print << solution_vars_.q_sol.segment(3+i*robot_model_.nq,4).squaredNorm() << ", ";
  }
  State_print << std::endl;
  State_print << "v measurement:\n" << guess_vars_.v_sol.head(robot_model_.nq).transpose() << std::endl;
  State_print << "v0 MPC last solution:\n" << solution_vars_.v_sol.head(robot_model_.nq).transpose() << std::endl;
  State_print << "Predicted v MPC last solution:\n" << solution_vars_.v_sol.segment(robot_model_.nq,robot_model_.nq).transpose() << std::endl;
  State_print << "----------------------------------------------------------------------\n";

  static std::ofstream log_file ("log_state.csv");
  log_file << t_msec << ", ";
     log_file << guess_vars_.q_sol.head(3).transpose() << ", ";
  for (int i = 1; i < N_+1; i++){
    log_file << solution_vars_.q_sol.segment(i*robot_model_.nq,3).transpose() << ", ";
  }
  log_file << "\n";
  log_file.flush();


    static std::ofstream log_file_vel ("log_state_vel.csv");
  log_file_vel << t_msec << ", ";
     log_file_vel << guess_vars_.v_sol.head(3).transpose() << ", ";
  for (int i = 1; i < N_+1; i++){
    log_file_vel << solution_vars_.v_sol.segment(i*robot_model_.nv,3).transpose() << ", ";
  }
  log_file_vel << "\n";
  log_file_vel.flush();
  



  // std::cout << "Update q0: " << guess_vars_.q_sol.transpose() << std::endl;
  // std::cout << "Update dq0: " << guess_vars_.v_sol.transpose() << std::endl;
  // TODO: update the Gamma_vec_ wr.t the current and desired foot step
  std::fill(Gamma_vec_.begin(), Gamma_vec_.end(), 0.0);
  if (current.is_left_foot_support){
    std::fill(Gamma_vec_.begin(), Gamma_vec_.begin() + 2*N_, 1.0);
  }
  if (current.is_right_foot_support){
    std::fill(Gamma_vec_.begin() + 2*N_, Gamma_vec_.end(), 1.0);
  }

  std::cout << "Gamma_vec_: " << Gamma_vec_ << std::endl;

  for (int iterr = 0; iterr < 1; iterr ++){// in case tessting more iterrations
  const casadi_real* data_meas[8];
  data_meas[0] = guess_vars_.q_sol.data();
  data_meas[1] = guess_vars_.v_sol.data();
  data_meas[2] = guess_vars_.F_lsole_sol.data();
  data_meas[3] = guess_vars_.F_rsole_sol.data();
  data_meas[4] = guess_vars_.tau_sol.data();
  data_meas[5] = &dt;
  data_meas[6] = &params_.mu;
  data_meas[7] = Gamma_vec_.data();
  // TODO: first we must update the desired variables wr.t the time step
  update_CoM_desired(t_msec, walking_data, desired_vars_);
  const casadi_real* data_cost_Jacob[17];
  data_cost_Jacob[0] = guess_vars_.q_sol.data();
  data_cost_Jacob[1] = desired_vars_.q_sol.data();
  data_cost_Jacob[2] = guess_vars_.v_sol.data();
  data_cost_Jacob[3] = desired_vars_.v_sol.data();
  data_cost_Jacob[4] = guess_vars_.F_lsole_sol.data();
  data_cost_Jacob[5] = desired_vars_.F_lsole_sol.data();
  data_cost_Jacob[6] = guess_vars_.F_rsole_sol.data();
  data_cost_Jacob[7] = desired_vars_.F_rsole_sol.data();

  data_cost_Jacob[8] = &params_.weight_com_xy;
  data_cost_Jacob[9] = &params_.weight_com_z;
  data_cost_Jacob[10] = &params_.weight_torso;
  data_cost_Jacob[11] = &params_.weight_general_qj;
  data_cost_Jacob[12] = &params_.weight_general_vb;
  data_cost_Jacob[13] = &params_.weight_general_omega_b;
  data_cost_Jacob[14] = &params_.weight_general_v;
  data_cost_Jacob[15] = &params_.weight_contact_force_xy;
  data_cost_Jacob[16] = &params_.weight_contact_force_z;

  //std::cout << "Codegen fconstraint" << std::endl;
  eval_codegen(eval_Jacobian_cost_work, eval_Jacobian_cost, eval_Jacobian_cost_sparsity_out, data_cost_Jacob, csc_g_ );

  eval_codegen(f_total_constraint_work, f_total_constraint, f_total_constraint_sparsity_out, data_meas,csc_constraint_ );
  //std::cout << "csc_constrain nzero: " << csc_constraint_.nzeros <<  std::endl;
  eval_codegen(Jacob_f_total_constraint_work, Jacob_f_total_constraint, Jacob_f_total_constraint_sparsity_out, data_meas,csc_Jacob_constraint_ );

  //update_weigtht_Jacobian(params_, robot_model.nq, robot_model.nv, robot_model.nv-6, 6*n_contacts_);
  Eigen::VectorXd dense_constraint = cscToDenseVector(csc_constraint_);
  g_ = cscToDenseVector(csc_g_);
  // std::cout << "CSC constraint: \n" << std::endl;
  // for (long long i = 0; i < csc_constraint_.nzeros; i++){
  //   //std::cout << "Enter the loop" << std::endl;
  //   std::cout << csc_constraint_.data[i] << " ";// << std::endl;
  // }
  std::cout << std::endl;
  //std::cout << "Dense constraint: \n" << dense_constraint.transpose() << std::endl;
  Eigen::VectorXd l_g = Eigen::VectorXd::Zero(n_wbnmpc_equalities_+n_wbnmpc_inequalities_);
  Eigen::VectorXd u_g = Eigen::VectorXd::Zero(n_wbnmpc_equalities_+n_wbnmpc_inequalities_);

  // The upper part of the inequality constraint must be different from the lower part
  u_g.segment(0*robot_model.nq + 0*robot_model.nv + N_*(robot_model.nq+robot_model.nv+ 2*3* n_contacts_), n_wbnmpc_inequalities_).setConstant(1e3);
  u_g.segment(N_*(robot_model.nq+robot_model.nv+ 2*3*n_contacts_) + n_wbnmpc_inequalities_, 2*2*N_).setConstant(0.2);
  l_g.segment(N_*(robot_model.nq+robot_model.nv+ 2*3*n_contacts_) + n_wbnmpc_inequalities_, 2*2*N_).setConstant(-0.2);
  // u_g.tail(16*N_).setConstant(0.1);
  // l_g.tail(16*N_).setConstant(-0.1);
  //u_g.segment(0, robot_model.nq) = q;
  //u_g.segment(robot_model.nq, robot_model.nv) = qdot;
  //l_g.segment(0, robot_model.nq) = q;
  //l_g.segment(robot_model.nq, robot_model.nv) = qdot;

  u_g = u_g -  dense_constraint;
  //std::cout << "Upper bound g: \n" << u_g.transpose() << std::endl;
  l_g = l_g -  dense_constraint;
  //std::cout << "Lower bound g: \n" << l_g.transpose() << std::endl;
  // Solve the QP
  qpsolvers::CSCMatrix_params P_upper;
    P_upper.nrows = P_.nrows;
    P_upper.ncols = P_.ncols;
    P_upper.col_pointers.push_back(0);

    for (int j = 0; j < P_.nrows; ++j) {
        int col_start = P_.col_pointers[j];
        int col_end   = P_.col_pointers[j+1];

        for (int k = col_start; k < col_end; ++k) {
            if (P_.row_indices[k] <= j) {          // keep only upper-triangular
                P_upper.row_indices.push_back(P_.row_indices[k]);
                P_upper.data.push_back(P_.data[k]);
            }
        }
        P_upper.col_pointers.push_back(P_upper.row_indices.size());
    }
  P_upper.nzeros = P_upper.row_indices.size();

  wbnmpc_solver_ptr_->solve_CCS(P_upper,g_,csc_Jacob_constraint_,l_g,u_g);
  // Get the solution
  //double* sol = wbnmpc_solver_ptr_->get_solution();
  Eigen::Map<Eigen::VectorXd> solution(wbnmpc_solver_ptr_->get_solution().data(), n_wbnmpc_variables_);
  //std::cout << "Guess vars input to linesearch: \n" << guess_vars_.q_sol.transpose() << std::endl;
  solution_vars_ = backtracking_line_search(solution, guess_vars_, desired_vars_, robot_model_, dt, Gamma_vec_, params_,t_msec); 
  guess_vars_ = solution_vars_;
  
  data_meas[0] = guess_vars_.q_sol.data();
  data_meas[1] = guess_vars_.v_sol.data();
  data_meas[2] = guess_vars_.F_lsole_sol.data();
  data_meas[3] = guess_vars_.F_rsole_sol.data();
  data_meas[4] = guess_vars_.tau_sol.data();
  data_meas[5] = &dt;
  data_meas[6] = &params_.mu;
  data_meas[7] = Gamma_vec_.data();
  eval_codegen(f_total_constraint_work, f_total_constraint, f_total_constraint_sparsity_out, data_meas,csc_constraint_ );
  dense_constraint = cscToDenseVector(csc_constraint_);
  //std::cout << "Dense constraint after linesearch: \n" << dense_constraint.transpose() << std::endl;

  }



  }

  // Update the guess_vars_ for the next iteration
  //guess_vars_ = solution_vars_;

  //std::cout<< "OSQP torque solution: \n" << solution_vars_.tau_sol.segment(0,n_joints_).transpose() << std::endl;

  // update the guess_vars = gues_vars_ + \alpha * solution
  
  //std::cout << "OSQP torque solution: \n";
  JointCommand joint_command;
  for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model.njoints; ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    joint_command[joint_name] = solution_vars_.tau_sol[joint_id - 2];
    std::cout << "Joint: " << joint_name << " Command: " << joint_command[joint_name] << std::endl;
  }
  
  //pressAnyKey();
  return joint_command;
}

  void WholeBodyMPC::eval_codegen(  int (*fname_work)(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w),
                    int (*fname)(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem),
                    const casadi_int* (*fname_sparsity_out)(casadi_int i),
                    const casadi_real** data_in, qpsolvers::CSCMatrix_params &csc_out){

  casadi_int sz_arg_, sz_res_, sz_iw_, sz_w_;
  //CSCMatrix_params csc_out;
  


  //std::cout << "Calculating work size..." << std::endl;
  int work = fname_work(&sz_arg_, &sz_res_, &sz_iw_, &sz_w_);
  if (work > 0) {
    std::cerr << "Error in work size calculation: " << work << std::endl;
    return;
  }

  //const casadi_real *arg[*sz_arg_];
  // Extract the sparsity pattern https://github.com/casadi/casadi/issues/3701
  
  const casadi_int* sparsity = fname_sparsity_out(0);
  const casadi_int* colind = sparsity + 2;
  
  csc_out.nrows = sparsity[0];
  csc_out.ncols = sparsity[1];
  csc_out.nzeros = sparsity[csc_out.ncols + 2];

  csc_out.row_indices.resize(csc_out.nzeros);
  csc_out.col_pointers.resize(csc_out.ncols + 1);

  for (casadi_int i = 0; i <= csc_out.ncols; ++i){

    //std::cout << "colind["<< i << "] = " << colind[i] << std::endl;
    csc_out.col_pointers[i] = colind[i];
  }
  const casadi_int* rowind = colind + csc_out.ncols + 1;
  //std::cout << "rowind["<< 0 << "] = " << rowind[0] << std::endl;
  for (casadi_int i = 0; i < csc_out.nzeros; ++i){
    //std::cout << "rowind["<< i << "] = " << rowind[i] << std::endl;
    csc_out.row_indices[i] = rowind[i];  
  }

  csc_out.data.resize(csc_out.nzeros);
  casadi_real *res[sz_res_] = {csc_out.data.data()};
  casadi_int iw[sz_iw_];
  casadi_real w[sz_w_];

  const casadi_real* arg[sz_arg_];

  //casadi_real* res[1];
  //res[0] = csc_out.data.data();
  //arg = data_in;
  //std::cout << "Data in size: " << sz_arg_ << std::endl;
  int eval = fname(data_in, res, iw, w,0);
  

  if (eval > 0) {
    std::cerr << "Error in evaluation: " << eval << std::endl;
    return;
  } 

  // for(int i = 0; i < csc_out.nzeros; ++i){
  //   std::cout << "data " << i << ": " << csc_out.data[i] << std::endl;
  // };
  //std::cout<< "End of eval" << std::endl;
}



  void WholeBodyMPC::update_CoM_desired( int64_t t_msec, const labrob::WalkingData& walking_data, vars_WBNMPC &desired_vars){

    Eigen::Vector2d pre_mid_point;
    Eigen::Vector2d next_mid_point;
    Eigen::Vector2d current_CoM_pos = desired_vars.q_sol.head(2);
    static int64_t delta_t;
    //static int64_t t_0 = 0;
    std::cout << "Walking state: " << static_cast<int>(walking_data.getWalkingState()) << std::endl;

    if (walking_data.getWalkingState() == labrob::WalkingState::Init ||
        walking_data.getWalkingState() == labrob::WalkingState::Starting ||
        walking_data.getWalkingState() == labrob::WalkingState::PostureRegulation) {
        // keep the current CoM position
        delta_t = walking_data.footstep_plan[1].getDuration();
        std::cout << "Delta t starting: " << delta_t << std::endl;
        desired_vars.v_sol.head(3).setZero();
        // current_CoM_pos = 0.5*(walking_data.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration().p.head(2) +
        //                        walking_data.footstep_plan.front().getFeetPlacement().getRightFootConfiguration().p.head(2));

        //return;
    }
    else if( walking_data.getWalkingState() == labrob::WalkingState::SingleSupport){
      
      
       pre_mid_point = 0.5*(walking_data.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration().p.head(2) +
                                walking_data.footstep_plan.front().getFeetPlacement().getRightFootConfiguration().p.head(2));

      if (walking_data.footstep_plan.front().getFeetPlacement().getSupportFoot() == Foot::RIGHT ){
        // Left foot is swinging, and at the next DoubleSupport phae, new left foot placement is given
        next_mid_point = 0.5*(walking_data.footstep_plan[1].getFeetPlacement().getLeftFootConfiguration().p.head(2) +
                                walking_data.footstep_plan.front().getFeetPlacement().getRightFootConfiguration().p.head(2));  
        //TODO: This is not a proper way to upday t_0, this update must be called depends on the first support foot
        //t_0 = walking_data.t0;
      }
      else{
        // Right foot is swinging, and at the next DoubleSupport phae, new right foot placement is given
        next_mid_point = 0.5*(walking_data.footstep_plan[1].getFeetPlacement().getRightFootConfiguration().p.head(2)+
                        walking_data.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration().p.head(2));// Vector3d
      }
      
      desired_vars.v_sol.head(2) = (next_mid_point-pre_mid_point) / (static_cast<double> (delta_t) * 0.001); // in m/s

      //desired_vars.q_sol.head(2) = pre_mid_point + desired_vars.v_sol.head(2) * (static_cast<double> (t_msec - t_0) * 0.001); // in m
      std::cout << "Single support: " << std::endl;
       //pressAnyKey();
      
    }
    else if (walking_data.getWalkingState() == labrob::WalkingState::DoubleSupport){
      //t_0 = walking_data.t0; 
       delta_t = walking_data.footstep_plan.front().getDuration()+
                 walking_data.footstep_plan[1].getDuration();

      //pre_mid_point = 0.5*(walking_data.footstep_plan.front().getFeetPlacement().getLeftFootConfiguration().p.head(2) +
      //                          walking_data.footstep_plan.front().getFeetPlacement().getRightFootConfiguration().p.head(2));
      //desired_vars.q_sol.head(2) = pre_mid_point + desired_vars.v_sol.head(2) * (static_cast<double> (t_msec - t_0) * 0.001); // in m
      std::cout << "Double support: " << std::endl;
       //pressAnyKey();
    }

    desired_vars.q_sol.head(2) = current_CoM_pos + desired_vars.v_sol.head(2) * 0.001; // in m

    
    //std::cout << "t0: "<< t_0 << ", t_msec: " << t_msec << std::endl;
    // std::cout << "Pre mid point: \n" << pre_mid_point.transpose() << std::endl;
    // std::cout << "Next mid point: \n" << next_mid_point.transpose() << std::endl;
    // std::cout << "Desired velocity: \n" << desired_vars.v_sol.transpose() << std::endl;
    // std::cout << "Desired position: \n" << desired_vars.q_sol.transpose() << std::endl;
   

  }

vars_WBNMPC WholeBodyMPC::backtracking_line_search(
    const Eigen::VectorXd& delta_w,
    vars_WBNMPC& guess_vars,
    vars_WBNMPC& desired_vars,
    const pinocchio::Model& robot_model,
    double dt,
    const std::vector<double>& Gamma_vec,
    const WholeBodyMPCParams& params,
    int64_t t_msec,
    // line search parameters
    double alpha_min   ,
    // double theta_max   ,
    // double theta_min   ,
    // double eta         ,
    // double gamma_phi   ,
    // double gamma_theta ,
    double gamma_alpha 
    
) {
    // ========================
    // Helpers
    // ========================

    static std::ofstream Cost_constr ("Cost_Constr");

    static std::ofstream Input("Input");
    //std::cout << "Desired q sol:\n" << desired_vars.q_sol.transpose() << std::endl;
    Cost_constr << "Time: " << t_msec << " ms ------------------\n";
    Input << "Time: " << t_msec << " ms ------------------\n";
    auto evaluate_candidate = [&](const vars_WBNMPC& candidate_vars) {
        // vars_WBNMPC candidate_vars = guess_vars;
        // candidate_vars.fromEigen(w);

        // Constraints input
        const casadi_real* data_meas[8];
        data_meas[0] = candidate_vars.q_sol.data();
        data_meas[1] = candidate_vars.v_sol.data();
        data_meas[2] = candidate_vars.F_lsole_sol.data();
        data_meas[3] = candidate_vars.F_rsole_sol.data();
        data_meas[4] = candidate_vars.tau_sol.data();
        data_meas[5] = &dt;
        data_meas[6] = &params.mu;
        data_meas[7] = Gamma_vec.data();

        
        // Cost input
        const casadi_real* data_cost[17];
        data_cost[0]  = candidate_vars.q_sol.data();
        data_cost[1]  = desired_vars.q_sol.data();
        data_cost[2]  = candidate_vars.v_sol.data();
        data_cost[3]  = desired_vars.v_sol.data();
        data_cost[4]  = candidate_vars.F_lsole_sol.data();
        data_cost[5]  = desired_vars.F_lsole_sol.data();
        data_cost[6]  = candidate_vars.F_rsole_sol.data();
        data_cost[7]  = desired_vars.F_rsole_sol.data();
        data_cost[8]  = &params.weight_com_xy;
        data_cost[9]  = &params.weight_com_z;
        data_cost[10] = &params.weight_torso;
        data_cost[11] = &params.weight_general_qj;
        data_cost[12] = &params.weight_general_vb;
        data_cost[13] = &params.weight_general_omega_b;
        data_cost[14] = &params.weight_general_v;
        data_cost[15] = &params.weight_contact_force_xy;
        data_cost[16] = &params.weight_contact_force_z;

        // Evaluate cost
        eval_codegen(eval_cost_work, eval_cost, eval_cost_sparsity_out,
                     data_cost, csc_cost_);
        Eigen::VectorXd dense_cost = cscToDenseVector(csc_cost_);
        double phi_val = dense_cost.sum();

        // Evaluate constraints
        eval_codegen(f_total_constraint_work, f_total_constraint,
                     f_total_constraint_sparsity_out, data_meas, csc_constraint_);
        Eigen::VectorXd dense_constraint = cscToDenseVector(csc_constraint_);
        //std::cout << "Dense constraint kin sumsquare: " << dense_constraint.segment(0,N_ * robot_model.nq ).squaredNorm() << std::endl;
        //std::cout << "Dense constraint Dyn: " << dense_constraint.segment(N_ * robot_model.nq ,1 * robot_model.nv ).transpose() << std::endl;
        //std::cout << "Dyn constraint sumsquare: " << dense_constraint.segment(N_ * robot_model.nq ,N_ * robot_model.nv ).squaredNorm() << std::endl;

        //std::cout << "Dense constraint Fswing: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv), 2 * 2 * 3 * N_ ).transpose() << std::endl;
        // std::cout << "Dense constraint Fswing sumsquare: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv), 2 * 2 * 3 * N_ ).squaredNorm() << std::endl;
        // //std::cout << "Dense constraint Friction: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 ), 2 * 2 * N_ ).transpose() << std::endl;
        // std::cout << "Dense constraint Friction sumsquare: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 ), 2 * 2 * N_ ).squaredNorm() << std::endl;
        // //std::cout << "Dense constraint feet height: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 + 2 * 2 ), 4 * N_ ).transpose() << std::endl;
        // std::cout << "Dense constraint feet height sumsquare: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 + 2 * 2 ), 4 * N_ ).squaredNorm() << std::endl;        
        //std::cout << "Dense constraint tang contact vel: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 + 2 * 2 + 4), 8 * N_ ).transpose() << std::endl;
        //std::cout << "Dense constraint tang contact vel sumsquare: " << dense_constraint.segment(N_ * (robot_model.nq + robot_model.nv + 2 * 2 * 3 + 2 * 2 + 4), 8 * N_ ).squaredNorm() << std::endl;
        
        Cost_constr << "Dense constraint Dyn:\n"  << dense_constraint.segment(robot_model.nq*0 + robot_model.nv*0 +N_ * robot_model.nq ,N_ * robot_model.nv ).transpose() << std::endl;
        Cost_constr << "Dyn constraint sumsquare:"  << dense_constraint.segment(robot_model.nq*0 + robot_model.nv*0 +N_ * robot_model.nq ,N_ * robot_model.nv ).squaredNorm() << std::endl;

        //std::cout << "Dense constraint Dyn: " << dense_constraint.segment(N_ * robot_model.nq ,N_ * robot_model.nv ) << std::endl;
        // Minus the inequality constrains part
        double theta_val = dt*(dense_constraint.squaredNorm()-dense_constraint.segment(robot_model.nq*0 + robot_model.nv*0 +N_ * (robot_model.nq + robot_model.nv + 2 * 4 * 3 ), 2 * 4 * N_ ).squaredNorm());// 4 is the num_contact


        return std::make_pair(phi_val, theta_val);
    };

    auto compute_grad_phi = [&](const vars_WBNMPC& candidate_vars) {
        // labrob::GuessVars candidate_vars = guess_vars_;
        // candidate_vars.fromEigen(w);

        const casadi_real* data_cost_Jacob[17];
        data_cost_Jacob[0]  = candidate_vars.q_sol.data();
        data_cost_Jacob[1]  = desired_vars_.q_sol.data();
        data_cost_Jacob[2]  = candidate_vars.v_sol.data();
        data_cost_Jacob[3]  = desired_vars_.v_sol.data();
        data_cost_Jacob[4]  = candidate_vars.F_lsole_sol.data();
        data_cost_Jacob[5]  = desired_vars_.F_lsole_sol.data();
        data_cost_Jacob[6]  = candidate_vars.F_rsole_sol.data();
        data_cost_Jacob[7]  = desired_vars_.F_rsole_sol.data();
        data_cost_Jacob[8]  = &params_.weight_com_xy;
        data_cost_Jacob[9]  = &params_.weight_com_z;
        data_cost_Jacob[10]  = &params_.weight_torso;
        data_cost_Jacob[11] = &params_.weight_general_qj;
        data_cost_Jacob[12] = &params_.weight_general_vb;
        data_cost_Jacob[13] = &params_.weight_general_omega_b;
        data_cost_Jacob[14] = &params_.weight_general_v;
        data_cost_Jacob[15] = &params_.weight_contact_force_xy;
        data_cost_Jacob[16] = &params_.weight_contact_force_z;

        eval_codegen(eval_Jacobian_cost_work, eval_Jacobian_cost,
                     eval_Jacobian_cost_sparsity_out, data_cost_Jacob, csc_g_);
        return cscToDenseVector(csc_g_);
    };

    // ========================
    // Line search loop
    // ========================
    double alpha = 1.0;
    Eigen::VectorXd w_next;
    bool accepted = false;
    std::cout << "size of q guess: "<< guess_vars.q_sol.size() << std::endl;
    auto [phi_i, theta_i] = evaluate_candidate(guess_vars);

    
        
        // Adaptive parameters based on history
        double theta_max, theta_min, eta, gamma_phi, gamma_theta;
        
        // Check if constraints are consistently satisfied
        bool constraints_satisfied = true;
        for (double theta : constraint_history_) {
            if (theta > 10) {
                constraints_satisfied = false;
                break;
            }
        }
        
        // Check if cost is improving
        bool cost_improving = true;
        if (cost_history_.size() >= 3) {
            for (size_t i = 1; i < cost_history_.size(); i++) {
                if (cost_history_[i] > cost_history_[i-1] * 1.1) {
                    cost_improving = false;
                    break;
                }
            }
        }
        
        if (constraints_satisfied && cost_improving) {
            // COST PRIORITY MODE
            std::cout << "MODE: Cost Priority" << std::endl;
            theta_max = 1e1;     // Very tolerant of violations
            theta_min = 1e-4;    // Switch to cost mode easily
            eta = 1e-6;          // Accept small cost reductions
            gamma_phi = 1e-8;    // Very permissive on cost
            gamma_theta = 0.99;  // Need big constraint reduction to override cost
        }
        else if (!constraints_satisfied) {
            // CONSTRAINT PRIORITY MODE
            std::cout << "MODE: Constraint Priority" << std::endl;
            theta_max = 1e-2;
            theta_min = 1e-6;
            eta = 1e-2;
            gamma_phi = 1e-4;
            gamma_theta = 0.5;   // Accept moderate constraint reductions
        }
        else {
            // BALANCED MODE
            std::cout << "MODE: Balanced" << std::endl;
            theta_max = 1e-1;
            theta_min = 1e-5;
            eta = 1e-4;
            gamma_phi = 1e-6;
            gamma_theta = 0.9;
        }

    //std::cout << "Current cost: " << phi_i << ", current constrain: " << theta_i << std::endl;
    
    Cost_constr <<"Current cost:" << phi_i << ", current constrain: " << theta_i << std::endl;
    //std::cout << "SQP solution : \n" << delta_w.transpose() << std::endl;
    Eigen::VectorXd grad_phi_wi = compute_grad_phi(guess_vars);
    double descent_dir = grad_phi_wi.dot(delta_w);
    vars_WBNMPC candidate_vars;
    candidate_vars.q_sol.resize(guess_vars.q_sol.size());
    candidate_vars.v_sol.resize(guess_vars.v_sol.size());
    candidate_vars.F_lsole_sol.resize(guess_vars.F_lsole_sol.size());
    candidate_vars.F_rsole_sol.resize(guess_vars.F_rsole_sol.size());
    candidate_vars.tau_sol.resize(guess_vars.tau_sol.size());

    candidate_vars.q_sol = guess_vars.q_sol;
    candidate_vars.v_sol = guess_vars.v_sol;

    int len_q = guess_vars.q_sol.size() - robot_model.nq;
    
    int len_v = guess_vars.v_sol.size() - robot_model.nv;
    // for (int i = 0; i< N_; i++){
    //   std::cout << "SQP solution for q, step " << i+1 << ": \n" << delta_w.segment(i*robot_model.nq, robot_model.nq).transpose() << std::endl;
    // }

    for (int i = 0; i< N_; i++){
      std::cout << "SQP solution for v, step " << i+1 << ": \n" << delta_w.segment(len_q+i*robot_model.nv, robot_model.nv).transpose() << std::endl;
    }
    //std::cout << "SQP solution for q: \n" << delta_w.head(len_q).transpose() << std::endl;
    //Update candidate vars, but not q0 and v0
    while (!accepted && alpha >= alpha_min) {
        candidate_vars.q_sol.segment(robot_model.nq, len_q) = 
            guess_vars.q_sol.segment(robot_model.nq, len_q) + alpha * delta_w.segment(0, len_q);
        //std::cout << "Guess q: \n" << guess_vars.q_sol.transpose() << std::endl;
        //std::cout << "Candidate q: \n" << candidate_vars.q_sol.transpose() << std::endl;
        candidate_vars.v_sol.segment(robot_model.nv, len_v) = 
            guess_vars.v_sol.segment(robot_model.nv, len_v)  + alpha * delta_w.segment(len_q, len_v);

        candidate_vars.F_lsole_sol = guess_vars.F_lsole_sol + alpha * delta_w.segment(len_q + len_v, guess_vars.F_lsole_sol.size());
        candidate_vars.F_rsole_sol = guess_vars.F_rsole_sol + alpha * delta_w.segment(len_q + len_v + guess_vars.F_lsole_sol.size(), guess_vars.F_rsole_sol.size());
        candidate_vars.tau_sol     = guess_vars.tau_sol     + alpha * delta_w.segment(len_q + len_v + guess_vars.F_lsole_sol.size() + guess_vars.F_rsole_sol.size(), guess_vars.tau_sol.size());
        Cost_constr << "alpha: " << alpha << std::endl;
        auto [phi_next, theta_next] = evaluate_candidate(candidate_vars);
        //std::cout << "Next cost: " << phi_next << ", next constrain: " << theta_next << ", alpha: " << alpha << std::endl;

        Cost_constr <<"Next cost:" << phi_next << ", next constrain: " << theta_next <<std::endl;
        if (theta_next > theta_max) {
            if (theta_next < (1 - gamma_theta) * theta_i) {
                accepted = true;
                //w_next = candidate;
            }
        }
        else if (std::max(theta_next, theta_i) < theta_min && descent_dir < 0) {
            if (phi_next < phi_i + eta * alpha*descent_dir) {
                accepted = true;
                //w_next = candidate;
            }
        }
        else if (phi_next < phi_i - gamma_phi * theta_i ||
                 theta_next < (1 - gamma_theta) * theta_i) {
            accepted = true;
            //w_next = candidate;
        }

        if (!accepted) alpha *= gamma_alpha;
    }
    std::cout << "Line search alpha: " << alpha << ", accepted: " << accepted << std::endl;
    Cost_constr << "Line search alpha: " << alpha << ", accepted: " << accepted << std::endl;
    Cost_constr << "---------------------------------------------------------\n";

    
    //std::cout << "Torque guess: \n" << guess_vars.tau_sol.head(n_joints_).transpose() << std::endl;
    //std::cout << "Delta torque: \n" << delta_w.segment(guess_vars.q_sol.size() + guess_vars.v_sol.size() + guess_vars.F_lsole_sol.size() + guess_vars.F_rsole_sol.size(), guess_vars.tau_sol.size()).head(n_joints_).transpose() << std::endl;
    //std::cout << "Torque candidate: \n" << candidate_vars.tau_sol.head(n_joints_).transpose() << std::endl;
    
    Input << "Torque guess: \n" << guess_vars.tau_sol.head(n_joints_).transpose() << std::endl;
    Input << "Torque candidate: \n" << candidate_vars.tau_sol.head(n_joints_).transpose() << std::endl;
    // std::cout << "Desired var q: "<< desired_vars.q_sol.transpose()<< std::endl;
    // std::cout << "Desired var v: "<< desired_vars.v_sol.transpose()<< std::endl;
    //std::cout << "Desired var F_l: "<< desired_vars.F_lsole_sol.transpose()<< std::endl;
    //std::cout << "Desired var F_r: "<< desired_vars.F_rsole_sol.transpose()<< std::endl;
    // std::cout << "Desired var tau: "<< desired_vars.tau_sol.transpose() << std::endl;
   
    
    
    //std::cout << "Guess var q: "<< guess_vars.q_sol.transpose()<< std::endl;
    // std::cout << "Guess var v: "<< guess_vars.v_sol.transpose()<< std::endl;
    // std::cout << "Guess var F_l: "<< guess_vars.F_lsole_sol.transpose()<< std::endl;
    // std::cout << "Guess var F_r: "<< guess_vars.F_rsole_sol.transpose()<< std::endl;
    // std::cout << "Guess var tau: "<< guess_vars.tau_sol.transpose() << std::endl;
    Input << "Guess var F_l: \n" << guess_vars.F_lsole_sol.transpose()<< std::endl;
    Input << "Guess var F_r: \n" << guess_vars.F_rsole_sol.transpose()<< std::endl;

    // std::cout << "Delta Force left: " << delta_w.segment(guess_vars.q_sol.size()+guess_vars.v_sol.size(),guess_vars.F_lsole_sol.size()).transpose() <<std::endl;
    // std::cout << "Delta Force right: " << delta_w.segment(guess_vars.q_sol.size()+guess_vars.v_sol.size()+guess_vars.F_lsole_sol.size(),guess_vars.F_rsole_sol.size()).transpose() <<std::endl;

    //std::cout << "Candidate var q: "<< candidate_vars.q_sol.transpose()<< std::endl;
    //std::cout << "Candidate var v: "<< candidate_vars.v_sol.transpose()<< std::endl;
    // std::cout << "Candidate var F_l: "<< candidate_vars.F_lsole_sol.transpose()<< std::endl;
    // std::cout << "Candidate var F_r: "<< candidate_vars.F_rsole_sol.transpose()<< std::endl;
    Input << "Candidate var F_l: \n" << candidate_vars.F_lsole_sol.transpose()<< std::endl;
    Input << "Candidate var F_r: \n" << candidate_vars.F_rsole_sol.transpose()<< std::endl;
    Input << "--------------------------------------------- \n";
    // std::cout << "Candidate var tau: "<< candidate_vars.tau_sol.transpose() << std::endl;
    auto [phi_next, theta_next] = evaluate_candidate(candidate_vars);
    // Update history
        cost_history_.push_back(phi_next);
        constraint_history_.push_back(theta_next);
        if (cost_history_.size() > 10) {
            cost_history_.pop_front();
            constraint_history_.pop_front();
        }


    
    return accepted ? candidate_vars : guess_vars;
}

} // end namespace labrob