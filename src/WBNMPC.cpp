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
  params.weight_com = 0.1; // First 3 components of vector q
  params.weight_torso = 1e-3; // The next fourth quaternion component of vector q
  params.weight_general_qj = 1e-3; // The rest of joint position

  params.weight_general_vb = 0.2; // The first three components of vector v (linear velocity of the base)
  params.weight_general_omega_b = 1e-3; // The next three components of vector v (angular velocity of the base)
  params.weight_general_v = 1e-3; // The rest of joint velocity

  params.weight_contact_force_xy = 4e-3; // Contact forces in x and y direction
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
    const Eigen::VectorXd& q_jnt_reg,
    double dynamic_discretization_time,
    int N,
    std::map<std::string, double>& armatures)
    : robot_model_(robot_model),
      q_jnt_reg_(q_jnt_reg),
      dynamic_discretization_time_(dynamic_discretization_time),
      N_(N),
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

  int num_q = robot_model_.nq*(N+1); // including q0
  int num_v = robot_model_.nv*(N+1); // including v0
  int num_contact = 2; // number of contacts per foot (Heel and Toe)
  int num_force = num_contact*3* N; // consider 1 foot, 3 forces per contact (Fx, Fy, Fz), N time steps
  int num_torques_single_step = robot_model_.nq-7; // excluding q0, q1, q2, q3, q4, q5, q6 (base link)
  int num_torques = num_torques_single_step*(N); // joint torques


  int num_q_single_step = robot_model_.nq; // = 28
  int num_v_single_step = robot_model_.nv; // = 27


  int num_force_single_foot_single_step = num_contact * 3; // 3 forces (Fx, Fy, Fz) per contact

  int num_constraint = N* ( num_q_single_step + // f_kin
                            num_v_single_step + // f_dyn
                            num_force_single_foot_single_step*2 + // f_Fswing
                            2*num_contact + // f_friction
                            2*num_contact + // f_feet_height
                            4*num_contact); // f_tang_contact_vel

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

  update_weight_Hessian(params_, num_q_single_step, num_v_single_step, num_torques_single_step, num_force_single_foot_single_step*2);

  std::cout << "Init OSQP solver" << std::endl;
  wbnmpc_solver_ptr_ = std::make_unique<qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<qpsolvers::OSQPSolver>(
          n_wbnmpc_variables_, n_wbnmpc_equalities_, n_wbnmpc_inequalities_,P_
      )
  );



  pressAnyKey();
  std::cout << "Space bar pressed! Program continuing." << std::endl;
  // Dict codegen_options;
  // codegen_options["with_header"] = true;
  // eval_Jacob_rnea_.generate("eval_Jacob_rnea", codegen_options);

  // Intialize the size  for the guess vars

  guess_vars_.q_sol.resize((size_t)num_q);
  guess_vars_.v_sol.resize((size_t)num_v);
  guess_vars_.F_lsole_sol.resize((size_t)num_force);
  guess_vars_.F_rsole_sol.resize((size_t)num_force);
  guess_vars_.tau_sol.resize((size_t)num_torques);

  //wbnmpc_solver_ptr_->P_ =
  g_.resize(n_wbnmpc_variables_);
  g_.setZero();
  



}


void WholeBodyMPC::update_first_guess(const pinocchio::Model& robot_model,
                                            pinocchio::Data& robot_data,
                                      const Eigen::VectorXd& q_init, 
                                      const int prediction_horizon)
{

  double total_mass = 0.0;
    for (const auto &inertial : robot_model.inertias) {
        total_mass += inertial.mass();
    }
  pinocchio::FrameIndex lsole_idx = robot_model.getFrameId("left_foot_link");
  pinocchio::FrameIndex rsole_idx = robot_model.getFrameId("right_foot_link");

  Eigen::MatrixXd J_lsole(6, robot_model.nv);
  Eigen::MatrixXd J_rsole(6, robot_model.nv);
  //pinocchio::computeJointJacobians(robot_model, robot_data, q_init);
  
  pinocchio::getFrameJacobian(
        robot_model,
        robot_data,
        lsole_idx,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_lsole
    );

  pinocchio::getFrameJacobian(
        robot_model,
        robot_data,
        rsole_idx,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J_rsole
    );


  Eigen::VectorXd qdot_init = Eigen::VectorXd::Zero(robot_model.nv);
  Eigen::VectorXd qddot_init = Eigen::VectorXd::Zero(robot_model.nv);
  Eigen::VectorXd F_lsole_init = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd F_rsole_init = Eigen::VectorXd::Zero(6);
  F_lsole_init(2) = total_mass * 9.81 / 4.0; // Quarter of the weight on each contact point/foot
  F_lsole_init(5) = total_mass * 9.81 / 4.0; // Quarter of the weight on each contact point/foot
  F_rsole_init(2) = total_mass * 9.81 / 4.0; // Quarter of the weight on each contact point/foot
  F_rsole_init(5) = total_mass * 9.81 / 4.0; // Quarter of the weight on each contact point/foot

  pinocchio::rnea(robot_model, robot_data, q_init, qdot_init, qddot_init);

  Eigen::VectorXd tau_init = Eigen::VectorXd::Zero(robot_model.nv);
  Eigen::Map<pinocchio::Model::TangentVectorType>(tau_init.data(), robot_model.nv, 1) = robot_data.tau;
  std::cout << "Initial tau from RNEA: \n" << tau_init.transpose() << std::endl;

  Eigen::Vector3d F_lsole_heel_init = F_lsole_init.head(3);
  Eigen::Vector3d F_lsole_toe_init = F_lsole_init.tail(3);
  Eigen::Vector3d F_rsole_heel_init = F_rsole_init.head(3);
  Eigen::Vector3d F_rsole_toe_init = F_rsole_init.tail(3);

  Eigen::Vector3d toe_pos_vec = Eigen::Vector3d(params_.foot_length/2, 0.0, 0.0); // Toe position in the local frame
  Eigen::Vector3d heel_pos_vec = Eigen::Vector3d(-params_.foot_length/2, 0.0, 0.0); // Heel position in the local frame

  Eigen::VectorXd wrench_lsole_init(6);
  Eigen::VectorXd wrench_rsole_init(6);
  wrench_lsole_init.head(3) = F_lsole_heel_init + F_lsole_toe_init;
  wrench_lsole_init.tail(3) = heel_pos_vec.cross(F_lsole_heel_init) + toe_pos_vec.cross(F_lsole_toe_init);

  wrench_rsole_init.head(3) = F_rsole_heel_init + F_rsole_toe_init;
  wrench_rsole_init.tail(3) = heel_pos_vec.cross(F_rsole_heel_init) + toe_pos_vec.cross(F_rsole_toe_init);

  tau_init = tau_init - J_lsole.transpose() * wrench_lsole_init - J_rsole.transpose() * wrench_rsole_init;
  // tau_init.segment(0,6) =  - wrench_lsole_init;
  // tau_init.segment(6,6) =  - wrench_rsole_init;
  std::cout << "Initial tau after considering contact forces: \n" << tau_init.transpose() << std::endl;


    guess_vars_.q_sol = q_init.replicate(prediction_horizon+1,1);
    guess_vars_.v_sol = qdot_init.replicate(prediction_horizon+1,1);
    guess_vars_.F_lsole_sol = F_lsole_init.replicate(prediction_horizon,1);
    guess_vars_.F_rsole_sol = F_rsole_init.replicate(prediction_horizon,1);
    guess_vars_.tau_sol = tau_init.tail(robot_model.nv-6).replicate(prediction_horizon,1);
  // for (int i = 0; i < prediction_horizon; i++){
  //   //Eigen::Map<pinocchio::Model::ConfigVectorType>(guess_vars_.q_sol.data()+(i+1)*robot_model.nq, robot_model.nq, 1) = q_init;
  //   Eigen::Map<pinocchio::Model::TangentVectorType>(guess_vars_.v_sol.data()+(i+1)*robot_model.nv, robot_model.nv, 1) = Eigen::VectorXd::Zero(robot_model.nv);
  //   Eigen::Map<pinocchio::Model::TangentVectorType>(guess_vars_.F_lsole_sol.data()+i*6, 6, 1) = F_lsole_init;
  //   Eigen::Map<pinocchio::Model::TangentVectorType>(guess_vars_.F_rsole_sol.data()+i*6, 6, 1) = F_rsole_init;
  //   Eigen::Map<pinocchio::Model::TangentVectorType>(guess_vars_.tau_sol.data()+i*(robot_model.nv-6), robot_model.nv-6, 1) = tau_init.tail(robot_model.nv-6);
  // };

  std:: cout << "Updated first guess q_sol: \n" << guess_vars_.q_sol.transpose() << std::endl;
  std:: cout << "Updated first guess v_sol: \n" << guess_vars_.v_sol.transpose() << std::endl;
  std:: cout << "Updated first guess F_lsole_sol: \n" << guess_vars_.F_lsole_sol.transpose() << std::endl;
  std:: cout << "Updated first guess F_rsole_sol: \n" << guess_vars_.F_rsole_sol.transpose() << std::endl;
  std:: cout << "Updated first guess tau_sol: \n" << guess_vars_.tau_sol.transpose() << std::endl;

  Gamma_vec_ = std::vector<double>(prediction_horizon * 4,1.0); // All contacts are active in the first guess
  
  desired_vars_ = guess_vars_;
  solution_vars_ = guess_vars_;

  

  pressAnyKey();

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
  double dt = 0.001;

  auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
  auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);

  // Update for the q0 and v0
  guess_vars_.q_sol.head(robot_model.nq) = q;
  guess_vars_.v_sol.head(robot_model.nv) = qdot;
  // std::copy( q.data(), q.data() + robot_model.nq, guess_vars_.q_sol.begin() );
  // std::copy( qdot.data(), qdot.data() + robot_model.nv, guess_vars_.v_sol.begin() );

  // TODO: update the Gamma_vec_ wr.t the current and desired foot step

  const casadi_real* data_meas[8];
  data_meas[0] = guess_vars_.q_sol.data();
  data_meas[1] = guess_vars_.v_sol.data();
  data_meas[2] = guess_vars_.F_lsole_sol.data();
  data_meas[3] = guess_vars_.F_rsole_sol.data();
  data_meas[4] = guess_vars_.tau_sol.data();
  data_meas[5] = &dt;
  data_meas[6] = &params_.mu;
  data_meas[7] = Gamma_vec_.data();
  //data_meas[8] = ref_feet_height_vec.data();
  //casadi_real* res_out[1];
  std::cout << "Codegen fconstraint" << std::endl;

  eval_codegen(f_total_constraint_work, f_total_constraint, f_total_constraint_sparsity_out, data_meas,csc_constraint_ );
  std::cout << "csc_constrain nzero: " << csc_constraint_.nzeros <<  std::endl;
  eval_codegen(Jacob_f_total_constraint_work, Jacob_f_total_constraint, Jacob_f_total_constraint_sparsity_out, data_meas,csc_Jacob_constraint_ );
  
  std::cout << "f_fconstraint_eval: " << std::endl;

  // TODO: first we must update the desired variables wr.t the time step
  

  update_weigtht_Jacobian(params_, robot_model.nq, robot_model.nv, robot_model.nv-6, 6*n_contacts_);
  Eigen::VectorXd dense_constraint = cscToDenseVector(csc_constraint_);
  // std::cout << "CSC constraint: \n" << std::endl;
  // for (long long i = 0; i < csc_constraint_.nzeros; i++){
  //   //std::cout << "Enter the loop" << std::endl;
  //   std::cout << csc_constraint_.data[i] << " ";// << std::endl;
  // }
  std::cout << std::endl;
  //std::cout << "Dense constraint: \n" << dense_constraint.transpose() << std::endl;
  Eigen::VectorXd l_g = Eigen::VectorXd::Zero(n_wbnmpc_variables_)- dense_constraint;
  Eigen::VectorXd u_g = Eigen::VectorXd::Zero(n_wbnmpc_variables_) - dense_constraint;

  // The upper part of the inequality constraint must be different from the lower part
  u_g.segment(N_*(robot_model.nq+robot_model.nv+6), n_wbnmpc_inequalities_).setConstant(1e10);

  // Solve the QP
  wbnmpc_solver_ptr_->solve_CCS(P_,g_,csc_Jacob_constraint_,l_g,u_g);
  //std::cout<< "OSQP solution: \n" << wbnmpc_solver_ptr_->get_solution().transpose() << std::endl;

  // update the guess_vars = gues_vars_ + \alpha * solution
  //pressAnyKey();


  

  JointCommand joint_command;

  return joint_command;
}

  void WholeBodyMPC::eval_codegen(  int (*fname_work)(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w),
                    int (*fname)(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem),
                    const casadi_int* (*fname_sparsity_out)(casadi_int i),
                    const casadi_real** data_in, qpsolvers::CSCMatrix_params &csc_out){

  casadi_int sz_arg_, sz_res_, sz_iw_, sz_w_;
  //CSCMatrix_params csc_out;



  std::cout << "Calculating work size..." << std::endl;
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
  std::cout << "rowind["<< 0 << "] = " << rowind[0] << std::endl;
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
  std::cout<< "End of eval" << std::endl;
}

  void WholeBodyMPC::update_weight_Hessian(WholeBodyMPCParams params, int num_q, int num_v, int num_torque, int num_force){
  //wbnmpc_solver_ptr_->solve_CSC()
  //int size = num_q + num_v +
    int num_var = num_q+num_v+num_force+num_torque;// var per step
    int num_qj = num_q -7;
    int num_vj = num_v -6;
    int num_q_v = num_q + num_v;
    std::cout << "Updating weight structure..." << std::endl;
    std::cout << "num_q: " << num_q << std::endl;
    std::cout << "num_v: " << num_v << std::endl;
    std::cout << "num_vars: " << num_var << std::endl;
    //int num_q_v_f = num_q_v + num_force_step;
    Eigen::VectorXd diag = Eigen::VectorXd::Zero(n_wbnmpc_variables_);

    for (int i = 0; i < N_; i++) {
        int offset = i * num_q;
        // q
        diag.segment(offset, 3).setConstant(params.weight_com);             // COM (3)
        diag.segment(offset + 3, 4).setConstant(params.weight_torso);       // Torso (4)
        diag.segment(offset + 7, num_qj).setConstant(params.weight_general_qj); // Joints qj
    }
    for (int i = 0; i < N_; i++){
        // v
        int v_off =  (N_+1)*num_q + i*num_v;
        diag.segment(v_off, 3).setConstant(params.weight_general_vb);       // Linear vel
        diag.segment(v_off + 3, 3).setConstant(params.weight_general_omega_b); // Angular vel
        diag.segment(v_off + 6, num_vj).setConstant(params.weight_general_v);   // Joint v
    }
    for (int i = 0; i < 4*N_; i++) {
        // forces (each contact: XY(2) + Z(1))
        int f_off =  (N_+1)*num_q_v + i*3;// one force includes x, y, z, i.e 3 components
         // 4 contacts: heel/toe left/right
            diag.segment(f_off, 2).setConstant(params.weight_contact_force_xy);// 3 is the size of each contact force vector
            diag(f_off + 2) = params.weight_contact_force_z;//2 is the offset caused by x,y components
        
    }
    std::cout << "Weight diag: \n" << diag.transpose() << std::endl;    

        // torques (uncomment if needed)
        // int tau_off = offset + num_q_v + num_force;
        // diag.segment(tau_off, num_torque).setConstant(params.weight_torque);

    // build diagonal matrix (if solver really needs a full MatrixXd)
    Eigen::MatrixXd P = diag.asDiagonal();
    std::cout << "Write Weight matrix P to file: \n" << std::endl;
    std::ofstream Weight_dense("Weight_P.txt");
    
    if (Weight_dense.is_open()) {
      Weight_dense << "Weight_P:\n";
      for (int i = 0; i < P.rows(); ++i) {
        for (int j = 0; j < P.cols(); ++j) {
            Weight_dense << std::fixed << std::setprecision(6) 
                         << std::setw(10) << P(i, j);
        }
        Weight_dense << "\n";
      }
    }
    P_ = qpsolvers::denseToCSC_param( P.data(), P.rows(), P.cols() );
  }

  void WholeBodyMPC::update_weigtht_Jacobian(WholeBodyMPCParams params, int num_q, int num_v, int num_torque, int num_force){
    int num_var = num_q+num_v+num_force+num_torque;// var per step
    int num_qj = num_q -7;
    int num_vj = num_v -6;
    int num_q_v = num_q + num_v;
    std::cout << "Updating weight structure for Jacobian..." << std::endl;
    std::cout << "num_q: " << num_q << std::endl;
    std::cout << "num_v: " << num_v << std::endl;
    std::cout << "num_vars: " << num_var << std::endl;
    //int num_q_v_f = num_q_v + num_force_step;
    for (int i = 0; i < N_; i++){
     int offset = i * num_q;
        // q terms
        g_.segment(offset, 3) = params.weight_com *
            (guess_vars_.q_sol.segment(offset, 3) - desired_vars_.q_sol.segment(offset, 3));

        g_.segment(offset + 3, 4) = params.weight_torso *
            (guess_vars_.q_sol.segment(offset + 3, 4) - desired_vars_.q_sol.segment(offset + 3, 4));

        g_.segment(offset + 7, num_qj) = params.weight_general_qj *
            (guess_vars_.q_sol.segment(offset + 7, num_qj) - desired_vars_.q_sol.segment(offset + 7, num_qj));
    }
        // v terms
    for (int i = 0; i < N_; i++){
        int v_off = (N_+1)*num_q + i*num_v;
        int var_v_off = i*num_v;

        g_.segment(v_off, 3) = params.weight_general_vb *
            (guess_vars_.v_sol.segment(var_v_off, 3) - desired_vars_.v_sol.segment(var_v_off, 3));

        g_.segment(v_off + 3, 3) = params.weight_general_omega_b *
            (guess_vars_.v_sol.segment(var_v_off + 3, 3) - desired_vars_.v_sol.segment(var_v_off + 3, 3));

        g_.segment(v_off + 6, num_vj) = params.weight_general_v *
            (guess_vars_.v_sol.segment(var_v_off + 6, num_vj) - desired_vars_.v_sol.segment(var_v_off + 6, num_vj));
    }
        // forces
    for (int i = 0; i < 2*N_; i++) { // 2 contacts per step (each foot)
        int fl_off = (N_+1)*num_q_v + i*3;
        int fr_off = (N_+1)*num_q_v + N_*6 +i*3; // second foot offset

            // XY part
            g_.segment(fl_off, 2) = params.weight_contact_force_xy *
                (guess_vars_.F_lsole_sol.segment(i*3, 2) - desired_vars_.F_lsole_sol.segment(i*3, 2));

            // Z part (scalar)
            g_(fl_off + 2) = params.weight_contact_force_z *
                (guess_vars_.F_lsole_sol(i*3 + 2) - desired_vars_.F_lsole_sol(i*3 + 2));

            // Right foot
            g_.segment(fr_off, 2) = params.weight_contact_force_xy *
                (guess_vars_.F_rsole_sol.segment(i*3, 2) - desired_vars_.F_rsole_sol.segment(i*3, 2));
            g_(fr_off + 2) = params.weight_contact_force_z *
                (guess_vars_.F_rsole_sol(i*3 + 2) - desired_vars_.F_rsole_sol(i*3 + 2));
    }  
    
  }

  void WholeBodyMPC::update_CoM_desired( int64_t t_msec, const labrob::WalkingData& walking_data, vars_WBNMPC &desired_vars){
    
  }

    

} // end namespace labrob