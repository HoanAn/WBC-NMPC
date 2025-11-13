// std
#include <fstream>
#include <memory>
#include <vector>
#include <chrono> 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <hrp4_locomotion/WalkingManager.hpp>
#include <labrob_qpsolvers/qpsolvers.hpp>

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
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <hrp4_locomotion/GaitConfiguration.hpp>
#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/RobotState.hpp>
#include <hrp4_locomotion/utils.hpp>
#include <hrp4_locomotion/eval_codegen_func.h>

#include "MujocoUI.hpp"

#include <casadi/casadi.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#define codegen_enabled 1

using namespace casadi;
using RobotModel = pinocchio::ModelTpl<double>;
using RobotData = pinocchio::DataTpl<double>;
using CasadiModel = pinocchio::ModelTpl<casadi::SX>;
using CasadiData = pinocchio::DataTpl<casadi::SX>;

using Ca_ConfigVector = CasadiData::ConfigVectorType;
using Ca_TangentVector = CasadiData::TangentVectorType;
using Ca_Matrix6x = CasadiData::Matrix6x;
using CasadiMotion = pinocchio::MotionTpl<casadi::SX>; 



std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> qp_solver_ptr_;

labrob::RobotState
robot_state_from_mujoco(mjModel* m, mjData* d) {
  labrob::RobotState robot_state;

  robot_state.position = Eigen::Vector3d(
    d->qpos[0], d->qpos[1], d->qpos[2]
  );

  robot_state.orientation = Eigen::Quaterniond(
      d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]
  );

  robot_state.linear_velocity = robot_state.orientation.toRotationMatrix().transpose() *
      Eigen::Vector3d(
          d->qvel[0], d->qvel[1], d->qvel[2]
      );

  robot_state.angular_velocity = Eigen::Vector3d(
    d->qvel[3], d->qvel[4], d->qvel[5]
  );

  for (int i = 1; i < m->njnt; ++i) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, i);
    robot_state.joint_state[name].pos = d->qpos[m->jnt_qposadr[i]];
    robot_state.joint_state[name].vel = d->qvel[m->jnt_dofadr[i]];
  }

  static double force[6];
  static double result[3];
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  robot_state.contact_points.resize(d->ncon);
  robot_state.contact_forces.resize(d->ncon);
  for (int i = 0; i < d->ncon; ++i) {
    mj_contactForce(m, d, i, force);
    //mju_rotVecMatT(result, force, d->contact[i].frame);
    mju_mulMatVec(result, d->contact[i].frame, force, 3, 3);
    for (int row = 0; row < 3; ++row) {
        result[row] = 0;
        for (int col = 0; col < 3; ++col) {
            result[row] += d->contact[i].frame[3 * col + row] * force[col];
        }
    }
    sum += Eigen::Vector3d(result);
    for (int j = 0; j < 3; ++j) {
      robot_state.contact_points[i](j) = d->contact[i].pos[j];
      robot_state.contact_forces[i](j) = result[j];
    }
  }

  robot_state.total_force = sum;

  return robot_state;
}

Eigen::MatrixXd convert_matrix_mujoco_to_eigen(mjtNum *matrix, int num_rows, int num_cols) {
  Eigen::MatrixXd result(num_rows, num_cols);
  for (int i = 0; i < num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
      result(i, j) = matrix[i * num_cols + j];
    }
  }
  return result;
}

struct codegen_params {
casadi_int n_in;
casadi_int n_out;
std::string in_id;
std::string out_id;
void* mem;
int work_return;
};

void eval_codegen(  int (*fname_work)(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w),
                    int (*fname)(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem),
                    const casadi_int* (*fname_sparsity_out)(casadi_int i),
                    const casadi_real** data_in, labrob::qpsolvers::CSCMatrix_params &csc_out){

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

SX quatMul(const SX& q1, const SX& q2) {
    SX x1 = q1(0), y1 = q1(1), z1 = q1(2), w1 = q1(3);
    SX x2 = q2(0), y2 = q2(1), z2 = q2(2), w2 = q2(3);
  
    return SX::vertcat({
        w1*x2 + x1*w2 + y1*z2 - z1*y2,   // x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,   // y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,   // z
        w1*w2 - x1*x2 - y1*y2 - z1*z2    // wBcz iBcaB
        });
}

    // Quaternion conjugate (inverse if unit quaternion)
SX quatConj(const SX& q) {
    return SX::vertcat({-q(0), -q(1), -q(2), q(3)});
}

  // Orientation error cost between q_ref and q_cur
SX quatErrorCost(const SX& q_ref, const SX& q_cur, SX weight) {
      SX q_err = quatMul(q_ref, quatConj(q_cur));  // relative quaternion
      SX v = q_err(Slice(0,3));                    // vector part [x,y,z]
      SX rotvec = 2 * v;                           // small-angle approx
      return weight * SX::dot(rotvec, rotvec);
}

struct cs_Weight{
  SX weight_com_xy = SX::sym("w_cxy",1);
  SX weight_com_z = SX::sym("w_cz",1);
  SX weight_torso = SX::sym("w_t",1);
  SX weight_general_qj = SX::sym("w_qj",1); //generalized configuration q = [p_b, \theta_b, q_j] \in R^{n_j +6}
  
  SX weight_general_vb = SX::sym("w_vb",1);
  SX weight_general_omega_b = SX::sym("w_om",1);
  SX weight_general_v = SX::sym("w_vj",1); //generalized velocity v = [vb ; \omega_b, \dot q_j]
  
  SX weight_contact_force_xy = SX::sym("w_fxy",1);
  SX weight_contact_force_z = SX::sym("w_fz",1);

  SX foot_length = 0.17;
  SX foot_width = 0.05;
};

int main() {
  // Load MJCF (for Mujoco):
  const int kErrorLength = 1024;          // load error string length
  char loadError[kErrorLength] = "";
  const char* mjcf_filepath = "../g1_mj_description/stair_steps.xml";
  mjModel* mj_model_ptr = mj_loadXML(mjcf_filepath, nullptr, loadError, kErrorLength);
  if (!mj_model_ptr) {
    std::cerr << "Error loading model: " << loadError << std::endl;
    return -1;
  }
  mjData* mj_data_ptr = mj_makeData(mj_model_ptr);

  std::ofstream joint_vel_log_file("/tmp/joint_vel.txt");// Writing data to a temp file
  std::ofstream joint_eff_log_file("/tmp/joint_eff.txt");
  std::ofstream joint_names_log_file("/tmp/joint_names.txt");

  // Init robot posture:
  mjtNum waist_p_init = 0.0;
  mjtNum waist_y_init = 0.0;
  mjtNum waist_r_init = 0.0;
  mjtNum r_hip_y_init = 0.0;
  mjtNum r_hip_r_init = -0.05;
  mjtNum r_hip_p_init = -0.44;
  mjtNum r_knee_init = 0.95;
  mjtNum r_ankle_p_init = -0.49;
  mjtNum r_ankle_r_init = 0.07;
  mjtNum l_hip_y_init = 0.0;
  mjtNum l_hip_r_init = -r_hip_r_init;
  mjtNum l_hip_p_init = r_hip_p_init;
  mjtNum l_knee_init = r_knee_init;
  mjtNum l_ankle_p_init = r_ankle_p_init;
  mjtNum l_ankle_r_init = -r_ankle_r_init;
  mjtNum r_shoulder_p_init = 0.07;
  mjtNum r_shoulder_r_init = -0.14;
  mjtNum r_shoulder_y_init = 0.0;
  mjtNum r_elbow_p_init = 3.14 / 2.0 - 0.44;
  mjtNum l_shoulder_p_init = r_shoulder_p_init;
  mjtNum l_shoulder_r_init = -r_shoulder_r_init;
  mjtNum l_shoulder_y_init = 0.0;
  mjtNum l_elbow_p_init = r_elbow_p_init;

  for (int i = 0; i < mj_model_ptr->nq; ++i) {
    mj_data_ptr->qpos[i] = 0.0;
  }
  // All the name between "" are the names of the joints in the MJCF (XML) file.
  // Update the initial posture to the Mujoco.
  mj_data_ptr->qpos[2] = 0.792151-0.125+0.0263 - 0.071;
  mj_data_ptr->qpos[3] = 1.0;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "waist_pitch_joint")]] = waist_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "waist_yaw_joint")]] = waist_y_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "waist_roll_joint")]] = waist_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_hip_yaw_joint")]] = r_hip_y_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_hip_roll_joint")]] = r_hip_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_hip_pitch_joint")]] = r_hip_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_knee_joint")]] = r_knee_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_ankle_pitch_joint")]] = r_ankle_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_ankle_roll_joint")]] = r_ankle_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_hip_yaw_joint")]] = l_hip_y_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_hip_roll_joint")]] = l_hip_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_hip_pitch_joint")]] = l_hip_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_knee_joint")]] = l_knee_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_ankle_pitch_joint")]] = l_ankle_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_ankle_roll_joint")]] = l_ankle_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_shoulder_pitch_joint")]] = r_shoulder_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_shoulder_roll_joint")]] = r_shoulder_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_shoulder_yaw_joint")]] = r_shoulder_y_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "right_elbow_joint")]] = r_elbow_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_shoulder_pitch_joint")]] = l_shoulder_p_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_shoulder_roll_joint")]] = l_shoulder_r_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_shoulder_yaw_joint")]] = l_shoulder_y_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "left_elbow_joint")]] = l_elbow_p_init;

  //print wolrd frame position
  std::cerr << "World frame position: " << mj_data_ptr->qpos[0] << " " << mj_data_ptr->qpos[1] << " " << mj_data_ptr->qpos[2] << std::endl;

  mjtNum* qpos0 = (mjtNum*) malloc(sizeof(mjtNum) * mj_model_ptr->nq);// dynamically asign memory for initial qpos
  memcpy(qpos0, mj_data_ptr->qpos, mj_model_ptr->nq * sizeof(mjtNum));// copy initial qpos defined above to qpos0

  std::vector<std::string> mujoco_locked_joints{
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_yaw_joint"
};

for (const auto& joint_name : mujoco_locked_joints) {
    int joint_id = mj_name2id(mj_model_ptr, mjOBJ_JOINT, joint_name.c_str());
    
    if (joint_id >= 0) {
        // Get the DOF address for this joint
        int dof_id = mj_model_ptr->jnt_dofadr[joint_id];
        
        // Store the initial position
        double locked_position = mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[joint_id]];
        
        // Set very high stiffness and damping to keep it fixed
        mj_model_ptr->dof_armature[dof_id] = 1000.0;  // High inertia
        
        // Option 1: Use position servo to hold position
        // Set the joint as a position-controlled actuator with very high gain
        
        // Option 2: Set joint limits to lock it
        int qpos_idx = mj_model_ptr->jnt_qposadr[joint_id];
        mj_model_ptr->jnt_range[joint_id * 2] = locked_position;      // lower limit
        mj_model_ptr->jnt_range[joint_id * 2 + 1] = locked_position;  // upper limit
        
        // Option 3: Zero out control and velocity
        mj_data_ptr->qvel[dof_id] = 0.0;
        
        std::cout << "Locked joint '" << joint_name 
                  << "' in Mujoco at position: " << locked_position << std::endl;
    } else {
        std::cerr << "Warning: Joint '" << joint_name << "' not found in Mujoco model" << std::endl;
    }
}
  // Create an array mapping joint names to their amateur value, 
  //amateur is a parameter that defines the resistance of the joint to motion, 
  //it is used in Mujoco to simulate the inertia of the joint.
  std::map<std::string, double> armatures;
  for (int i = 0; i < mj_model_ptr->nu; ++i) {
    int joint_id = mj_model_ptr->actuator_trnid[i * 2];// get the joint id from mujoco actuator
    std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));// get the joint name from mujoco, mjOBJ_JOINT is the type of object we are looking for id
    int dof_id = mj_model_ptr->jnt_dofadr[joint_id];
    //std::cerr << "DOF ID: " << dof_id << std::endl;
    armatures[joint_name] = mj_model_ptr->dof_armature[dof_id];
    //std::cerr << "Joint: " << joint_name << ", Armature: " << armatures[joint_name] << std::endl;
  }

  // Walking Manager:
  labrob::RobotState initial_robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
  // labrob::WalkingManager walking_manager;
  // walking_manager.init(initial_robot_state, armatures);
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
  // Define the locked joints in the constant string "joints_to_lock_names" 
  //for the pinocchio robot model to build a reduced model.
  const std::vector<std::string> joint_to_lock_names{"left_wrist_pitch_joint",
                                                     "left_wrist_roll_joint",
                                                     "left_wrist_yaw_joint",
                                                     "right_wrist_pitch_joint",
                                                     "right_wrist_roll_joint",
                                                     "right_wrist_yaw_joint"};
  //                                                    "left_hip_yaw_joint",
  //                                                     "right_hip_yaw_joint"
  //                                                    };//"left_hip_yaw_joint",

  //const std::vector<std::string> joint_to_lock_names{};//"left_hip_yaw_joint",                                                   
  std::vector<pinocchio::JointIndex> joint_ids_to_lock;
  for (const auto& joint_name : joint_to_lock_names) {
    if (full_robot_model.existJointName(joint_name)) {
      joint_ids_to_lock.push_back(full_robot_model.getJointId(joint_name));
    }
  }
  // Build such a rduced model by pinocchio API
  pinocchio::Model robot_model_test = pinocchio::buildReducedModel(
      full_robot_model,
      joint_ids_to_lock,
      pinocchio::neutral(full_robot_model)
  );
  pinocchio::Data robot_data_test = pinocchio::Data(robot_model_test);

  auto q_init = labrob::robot_state_to_pinocchio_joint_configuration(
      robot_model_test,
      initial_robot_state
  );
  labrob::RobotState robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
  std::cout << "Initial joint configuration measured from mujoco: " << std::endl;
  Eigen::VectorXd q_meas = labrob::robot_state_to_pinocchio_joint_configuration(
      robot_model_test,
      robot_state
  );
  std::cout << q_meas.transpose() << std::endl;

  Eigen::VectorXd v_meas = labrob::robot_state_to_pinocchio_joint_velocity(
      robot_model_test,
      robot_state
  );
  std::cout << "Joint velocity measured from mujoco: " << std::endl;
  std::cout << v_meas.transpose() << std::endl;

// --------------------Preparing casadi components------------------------------

  std::cout << "Setting up casadi components" << std::endl;

  int N = 25; // predicted horizion
  double d_dt = 0.001; // dynamic sample time - numerical value for testing
  double d_mu = 0.5; // friction coefficient - numerical value for testing

  double d_dt_arr[] = {d_dt};
  double d_mu_arr[] = {d_mu};


  int num_q = robot_model_test.nq*(N+1); // including q0
  int num_v = robot_model_test.nv*(N+1); // including v0
  int num_contact = 4; // number of contacts per foot (Heel and Toe)
  int num_force = num_contact*3* N; // consider 1 foot, 3 forces per contact (Fx, Fy, Fz), N time steps
  int num_torques_single_step = robot_model_test.nq-7; // excluding q0, q1, q2, q3, q4, q5, q6 (base link)
  int num_torques = num_torques_single_step*(N); // joint torques


  int num_q_single_step = robot_model_test.nq; // = 23
  int num_v_single_step = robot_model_test.nv; // = 22
  
  int num_force_single_foot_single_step = num_contact * 3; // 3 forces (Fx, Fy, Fz) per contact
  //int num_force = num_force_single_foot_single_step * N; // concatenated forces for N steps
  
  ///Preparing symbolic variables for Casadi

  SX dt = SX::sym("dt", 1); // dynamics discretization timestep

  SX ca_q = SX::sym("q", num_q); // position configuration vector for the whole horizon
  Ca_ConfigVector cs_q_k(num_q_single_step);

  SX ca_pos_q(robot_model_test.nq*N,1);
  ca_pos_q = ca_q( Slice(num_q_single_step, num_q_single_step * (N+1)) ); //next step position confiiguration, excluding q0 
  SX ca_q_r = SX::sym("q_r", num_q); // reference joint configuration (excluding q0)
 
//------------------------------------------------------------------------------------- 
  SX ca_v = SX::sym("v", num_v); // velocity vector for the whole horizon
  Ca_TangentVector cs_v_k(num_v_single_step);

  SX ca_pos_v(robot_model_test.nv*N,1);
  ca_pos_v = ca_v( Slice(num_v_single_step, num_v_single_step * (N+1)) ); // excluding the first component
  SX ca_v_r = SX::sym("v_r", num_v); // reference joint velocity

  std::cout << "ca_pos_v size: " << ca_pos_v.size() << std::endl;

  //labrob::pressAnyKey();
//-------------------------------------------------------------------------------------
  SX ca_v_excluding_final = ca_v(Slice(0, num_v_single_step * N)); // excluding the final velocity at the end of the horizon
  SX ca_a = (ca_pos_v - ca_v_excluding_final)/ dt;
  std::cout << "ca_a size: " << ca_a << std::endl;
  Ca_TangentVector cs_a_k(num_v_single_step);

//--------------------------------------------------------------------------------------  
  SX F_lsole = SX::sym("F_l", num_force); // left foot contact forces
  SX F_rsole = SX::sym("F_r", num_force); // right foot contact forces
  SX F_lsole_r = SX::sym("F_l_r", num_force); // Reference left foot contact forces
  SX F_rsole_r = SX::sym("F_r_r", num_force); // Reference right foot contact forces

  SX tau = SX::sym("tau", num_torques); // joint torques

  //----------------------THINK ABOUT IT-------------------------------------------------------------------------
  SX decision_vars = vertcat(SXVector{ca_pos_q, ca_pos_v, F_lsole, F_rsole, tau});// optimization decision variables
  //-------------------------------------------------------------------------------------------------------------
  //int num_gamma = num_contact * 2 *
  //Gamma are variables that captures the contact status
  SX Gam_h_l = SX::sym("g1l",N);
  SX Gam_t_l = SX::sym("g2l",N);
  SX Gam_h_r = SX::sym("g1r",N);
  SX Gam_t_r = SX::sym("g2r",N);

  SX Gamma = SX::vertcat({
    Gam_h_l, // Heel contact status left foot
    Gam_t_l, // Toe contact status left foot
    Gam_h_r, // Heel contact status right foot
    Gam_t_r  // Toe contact status right foot
  }); // Contact status variables

  SX mu = SX::sym("mu",1);

  

  /// Setup pinoccio elements to calculate cost function, kinematic and dynamic constraints in symbolic form
  CasadiModel model_casadi = robot_model_test.cast<SX>();
  CasadiData data_casadi(model_casadi);

  SX cost = 0.0; // cost function
  //cost = 0.0; // Initialize cost to zero
  cs_Weight WBNMPC_params;

  ///INITIAL CONSTRAINTS--------------------------------------------------
  SX f0 (num_q_single_step + num_v_single_step,1); // Initial constraints
  f0(Slice(0,num_q_single_step)) = ca_q(Slice(0, num_q_single_step)); // Meassured position at the update instance of the controller
  f0(Slice(num_q_single_step, num_q_single_step + num_v_single_step)) = ca_v(Slice(0, num_v_single_step)); // Meassured velocity at the update instance of the controller

  /// Kinematic constraints------------------------------------------------
  SX f_kin( (N-0)*num_q_single_step, 1); // N Kinematic constraints q(k+1) = q(k) + 0.5( qdot(k+1)+qdot(k) )*dt ,\forall k= 0 to N-1
  SX f_kin_i(num_q_single_step, 1);

  /// Dynamic constraints-------------------------------


  Ca_Matrix6x J_lsole_ca(6, model_casadi.nv);
  //J_lsole_ca.setOnes();
  Ca_Matrix6x J_rsole_ca(6, model_casadi.nv);
  //J_rsole_ca.setOnes();

  SX cs_v_lsole(6,1);
  SX cs_v_rsole(6,1);

  pinocchio::FrameIndex lsole_idx = model_casadi.getFrameId("left_foot_link");
  pinocchio::FrameIndex rsole_idx = model_casadi.getFrameId("right_foot_link");
  pinocchio::FrameIndex pelvis_idx = model_casadi.getFrameId("pelvis");


  SX f_dyn(N*num_v_single_step,1);
  SX f_dyn_i(num_v_single_step,1);
  SX tau_RNEA( num_v_single_step, 1); // RNEA torques for the current step
  //SX tau_i(num_torques_single_step, 1); // joint torques for the current step

  
  /// Contact and swing constraint------------------------------------------------------------
  SX f_Fswing (num_force_single_foot_single_step*2*N,1 );// 2 feet
  //SX f_Fswing_i( num_force_single_foot_single_step*2,1 );
  
  SX f_friction (num_contact*2*N,1);
  //SX f_friction_i (num_contact*2,1);

  SX f_feet_height (2*2*N,1);

  // Reference feet height
  SX ref_toe_l_height   = SX::sym("tlh_r",N);
  SX ref_heel_l_height  = SX::sym("hlh_r",N);
  SX ref_toe_r_height   = SX::sym("trh_r",N);
  SX ref_heel_r_height  = SX::sym("hrh_r",N);

  SX ref_feet_heiht = SX::vertcat({
    ref_toe_l_height,
    ref_heel_l_height,
    ref_toe_r_height,
    ref_heel_r_height
  });

  // tangential contact vel
  SX f_tang_contact_vel ( num_contact*2*2*N,1 );// x, y directions

  //
  int num_constraint =  N* (num_q_single_step + num_v_single_step + num_force_single_foot_single_step*2 + 2*num_contact + 2*2 + num_contact*2*2);
  // f_kin + f_dyn + f_swing + f_friction + f_contact_height (2*(heel+toe)) + f_tang_contact_vel
  //um_constraint = N* (num_q_single_step  + num_force_single_foot_single_step*2+ 2*num_contact + 2*num_contact+4*num_contact);
  
  std::cout << "Total number of constraints: " << num_constraint << std::endl;

  SX cs_f_total_constraint_pre (num_constraint);

  labrob::pressAnyKey();


//-----------------------------------------------------------------------------------------
// Building constraints at each time step k

  for (int k = 0; k < N; ++k) {
    // --- Extract k-th step variables ---
    std::cout << "Extracting k-th step variables for k = " << k << std::endl;
    // Extract the portion of the concatenated vectors corresponding to time step k
    SX q_k = ca_q(Slice( (k) * num_q_single_step, (k + 1) * num_q_single_step));
    SX q_r_k = ca_q_r(Slice( (k) * num_q_single_step, (k + 1) * num_q_single_step));

    SX q_k1 = ca_pos_q(Slice((k ) * num_q_single_step, (k + 1) * num_q_single_step));
    std::cout << "q_k1 size: " << q_k1.size() << std::endl;

    SX v_k = ca_v(Slice((k) * num_v_single_step, (k + 1) * num_v_single_step));
    SX v_r_k = ca_v_r(Slice((k) * num_v_single_step, (k + 1) * num_v_single_step));

    SX v_k1 = ca_pos_v(Slice((k ) * num_v_single_step, (k + 1) * num_v_single_step));
    SX a_k = ca_a(Slice((k) * num_v_single_step, (k + 1) * num_v_single_step));

    SX F_lsole_k = F_lsole(Slice(k * num_force_single_foot_single_step, (k + 1) * num_force_single_foot_single_step));
    SX F_rsole_k = F_rsole(Slice(k * num_force_single_foot_single_step, (k + 1) * num_force_single_foot_single_step));
    SX F_lsole_r_k = F_lsole_r(Slice(k * num_force_single_foot_single_step, (k + 1) * num_force_single_foot_single_step));
    SX F_rsole_r_k = F_rsole_r(Slice(k * num_force_single_foot_single_step, (k + 1) * num_force_single_foot_single_step));

    SX tau_k = tau(Slice(k*num_torques_single_step, (k + 1) * num_torques_single_step));
    // --- Calculate deviations (delta terms) ---
    SX delta_v_k = v_k - v_r_k;
    SX delta_q_k = q_k - q_r_k;




    SX delta_F_lsole_k = F_lsole_k - F_lsole_r_k;
    SX delta_F_rsole_k = F_rsole_k - F_rsole_r_k;

    // --- Calculate squared weighted norms (l_k) ---
    // ||delta_v_k||_W_v^2 = delta_v_k.T * W_v * delta_v_k
    // For scalar weights, this is sum(weight * element^2)
    SX cost_v = WBNMPC_params.weight_general_vb       * SX::sumsqr(delta_v_k(Slice(0, 3)))+
                WBNMPC_params.weight_general_omega_b  * SX::sumsqr(delta_v_k(Slice(3, 6))) +
                WBNMPC_params.weight_general_v        * SX::sumsqr(delta_v_k(Slice(6, num_v_single_step)));
    
    SX cost_q = WBNMPC_params.weight_com_xy * SX::sumsqr(delta_q_k(Slice(0, 2))) +
                WBNMPC_params.weight_com_z * SX::sumsqr(delta_q_k(2)) +
                
                WBNMPC_params.weight_general_qj * SX::sumsqr(delta_q_k(Slice(7, num_q_single_step)));
    //WBNMPC_params.weight_torso * SX::sumsqr(delta_q_k(Slice(3, 7))) +
    // Sum over contacts C (here, left and right feet)
    casadi::SX cost_F =   WBNMPC_params.weight_contact_force_xy * ( SX::sumsqr(delta_F_lsole_k(Slice(0,2))) 
                                                                  + SX::sumsqr(delta_F_lsole_k(Slice(3,5))) 
                                                                  + SX::sumsqr(delta_F_lsole_k(Slice(6,8))) 
                                                                  + SX::sumsqr(delta_F_lsole_k(Slice(9,11)))
                                                                  ) 
                        + WBNMPC_params.weight_contact_force_z  * ( SX::sumsqr(delta_F_lsole_k(2)) 
                                                                  + SX::sumsqr(delta_F_lsole_k(5))
                                                                  + SX::sumsqr(delta_F_lsole_k(8))
                                                                  + SX::sumsqr(delta_F_lsole_k(11))
                                                                  ) 
                        + WBNMPC_params.weight_contact_force_xy * ( SX::sumsqr(delta_F_rsole_k(Slice(0,2))) 
                                                                  + SX::sumsqr(delta_F_rsole_k(Slice(3,5))) 
                                                                  + SX::sumsqr(delta_F_rsole_k(Slice(6,8)))
                                                                  + SX::sumsqr(delta_F_rsole_k(Slice(9,11)))
                                                                  )
                        + WBNMPC_params.weight_contact_force_z  * ( SX::sumsqr(delta_F_rsole_k(2)) 
                                                                  + SX::sumsqr(delta_F_rsole_k(5))
                                                                  + SX::sumsqr(delta_F_rsole_k(8))
                                                                  + SX::sumsqr(delta_F_rsole_k(11))
                                                                  );

    // Total running cost for this time step k
    casadi::SX running_cost_k = cost_v + cost_q + cost_F;

    // Accumulate to the total cost
    cost += running_cost_k;

    std::cout << "Running cost for step " << k << std::endl;
 
      
    SX p_k = q_k(Slice(0, 3)); // Position of the base in the world frame
    SX p_k1 = q_k1(Slice(0, 3)); // Position of the base in the world frame for the next step
    SX u_quarternion_k = q_k(Slice(3, 7)); // Quaternion orientation of the base
    SX u_quarternion_k1 = q_k1(Slice(3, 7)); // Quaternion orientation of the base for the next step
    SX ur_quarternion_k = q_r_k(Slice(3, 7)); // Ref Quaternion orientation of the base
    
    SX qj_k = q_k(Slice(7, num_q_single_step)); // Joint configuration for the current step
    SX qj_k1 = q_k1(Slice(7, num_q_single_step)); // Joint configuration for the next step
    // Damn, why are they so muchhhh
      
    SX vt_k = v_k(Slice(0, 3)); // Linear velocity (translation) of the base
    SX vt_k1 = v_k1(Slice(0, 3)); // Linear velocity (translation) of the base for the next step
    SX omega_k = v_k(Slice(3, 6)); // Angular velocity of the base
    SX omega_k1 = v_k1(Slice(3, 6)); // Angular velocity of the base for the next step
    SX qj_dot_k = v_k(Slice(6, num_v_single_step)); // Joint velocity for the current step
    SX qj_dot_k1 = v_k1(Slice(6, num_v_single_step)); // Joint velocity for the next step


    f_kin_i(Slice(0,3)) = p_k1 - p_k - 0.5*(vt_k1 + vt_k)*dt; // Position constraint
    f_kin_i(Slice(7,num_q_single_step)) = qj_k1 - qj_k - 0.5*(qj_dot_k1 + qj_dot_k) * dt; // Joint configuration constraint

      
    // Quaternion components
    SX x = u_quarternion_k(0);
    SX y = u_quarternion_k(1);
    SX z = u_quarternion_k(2);
    SX w = u_quarternion_k(3);

    SX x1 = u_quarternion_k1(0);
    SX y1 = u_quarternion_k1(1);
    SX z1 = u_quarternion_k1(2);
    SX w1 = u_quarternion_k1(3);


    // SX x_r = ur_quarternion_k(0);
    // SX y_r = ur_quarternion_k(1);
    // SX z_r = ur_quarternion_k(2);
    // SX w_r = ur_quarternion_k(3);


    

    SX cost_torso = quatErrorCost(ur_quarternion_k, u_quarternion_k, WBNMPC_params.weight_torso);

    cost += cost_torso; // Adding the torso orientation cost


    // Construct the B(q_k) matrix
    // Quarternion derivative is here, for the local body frame: https://arxiv.org/pdf/0811.2889
    SX B_qk = SX::zeros(4, 3);
    B_qk(0, 0) = w;  B_qk(0, 1) = -z;  B_qk(0, 2) = y;
    B_qk(1, 0) = z; B_qk(1, 1) = w;  B_qk(1, 2) = -x;
    B_qk(2, 0) = -y;  B_qk(2, 1) = x; B_qk(2, 2) = w;
    B_qk(3, 0) = -x; B_qk(3, 1) = -y; B_qk(3, 2) = -z;

    SX B_qk1 = SX::zeros(4, 3);
    B_qk1(0, 0) = w1;  B_qk1(0, 1) = -z1;  B_qk1(0, 2) = y1;
    B_qk1(1, 0) = z1; B_qk1(1, 1) = w1;  B_qk1(1, 2) = -x1;
    B_qk1(2, 0) = -y1;  B_qk1(2, 1) = x1; B_qk1(2, 2) = w1;
    B_qk1(3, 0) = -x1; B_qk1(3, 1) = -y1; B_qk1(3, 2) = -z1;

    // B_qk(0, 0) = -x; B_qk(0, 1) = -y; B_qk(0, 2) = -z;
    // B_qk(1, 0) =  w; B_qk(1, 1) = -z; B_qk(1, 2) =  y;
    // B_qk(2, 0) =  z; B_qk(2, 1) =  w; B_qk(2, 2) = -x;
    // B_qk(3, 0) = -y; B_qk(3, 1) =  x; B_qk(3, 2) =  w;

    f_kin_i(Slice(3, 7)) = u_quarternion_k1 - u_quarternion_k - 0.5* 0.5 * (mtimes (B_qk,omega_k) + mtimes (B_qk1,omega_k1))*dt; // Quaternion constraint
    //std::cout << "f_kin_i \n: " << f_kin_i << std::endl;
  // f_constraint kinematic
    f_kin(Slice(k*num_q_single_step, (k + 1)*num_q_single_step)) = f_kin_i; // Store the constraint for this step


    // --- DYNAMIC RNEA CONSTRAINT ---
    
    std::cout << "Setting RNEA components" << std::endl;

    cs_q_k = Eigen::Map<Ca_ConfigVector>(static_cast<std::vector<SX> > (q_k).data(), num_q_single_step,1);
    std::cout << "Done setting cs_q" << std::endl;
    cs_v_k = Eigen::Map<Ca_TangentVector>(static_cast<std::vector<SX> > (v_k).data(), num_v_single_step,1);
    cs_a_k = Eigen::Map<Ca_TangentVector>(static_cast<std::vector<SX> > (a_k).data(), num_v_single_step,1);
    
    rnea(model_casadi, data_casadi, cs_q_k, cs_v_k, cs_a_k);
    //std::cout << "tau_RNEA:" << std::endl;
    for (Eigen::DenseIndex i = 0; i < num_v_single_step; ++i) {
      //std:: cout << "Setting tau_RNEA[" << i << "]" << std::endl;
      tau_RNEA(i) = data_casadi.tau[i];
      //std::cout << "tau_RNEA[" << i << "] = " << tau_RNEA(i) << std::endl;
      //std::cout << "Done the tau_RNEA[" << i << "]" << std::endl;
    }
    
    std::cout << "Done building RNEA torque" << std::endl;
    

    // Next, calculate foot jacobian
    //computeJointJacobians(model_casadi, data_casadi, cs_q_k);
    pinocchio::forwardKinematics(model_casadi, data_casadi, cs_q_k, cs_v_k);
    pinocchio::framesForwardKinematics(model_casadi, data_casadi, cs_q_k);
    pinocchio::computeJointJacobians(model_casadi, data_casadi, cs_q_k);
    updateFramePlacements(model_casadi, data_casadi);

    std::cout << "Done forward kinematics" << std::endl;


    // CasadiMotion v_lsole_world = data_casadi.oMi[(size_t)lsole_idx].act(v_lsole_local);
    // CasadiMotion v_rsole_world = data_casadi.oMi[(size_t)rsole_idx].act(v_rsole_local);

    CasadiMotion v_lsole_local_world = pinocchio::getFrameVelocity(model_casadi, data_casadi, lsole_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    CasadiMotion v_rsole_local_world = pinocchio::getFrameVelocity(model_casadi, data_casadi, rsole_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);


    CasadiMotion v_lsole_world = pinocchio::getFrameVelocity(model_casadi, data_casadi, lsole_idx, pinocchio::ReferenceFrame::WORLD);
    CasadiMotion v_rsole_world = pinocchio::getFrameVelocity(model_casadi, data_casadi, rsole_idx, pinocchio::ReferenceFrame::WORLD);
    

    const auto T_lsole = data_casadi.oMf[lsole_idx];
    const auto T_rsole = data_casadi.oMf[rsole_idx];

    std::cout << "Done extracting foot vel" << std::endl;

    for (Eigen::DenseIndex i = 0; i <  6; ++i)
    {
      cs_v_lsole(i) = v_lsole_local_world.toVector()[i];
      cs_v_rsole(i) = v_rsole_local_world.toVector()[i];
      //std::cout << "cs_v_lsole(" << i << ") = " << cs_v_lsole(i) << std::endl;
    }
    
    SX J_lsole_cs = jacobian(cs_v_lsole, v_k);
    SX J_rsole_cs = jacobian(cs_v_rsole, v_k);

    std::cout << "Done calculating foot jacobian" << std::endl;

    std::cout << "J_lsole_cs size: ( " << J_lsole_cs.rows() << "," << J_lsole_cs.columns() << ")" << std::endl;

        // Get rotation matrices
    // const auto T_lsole = data_casadi.oMf[lsole_idx];
    // const auto T_rsole = data_casadi.oMf[rsole_idx];
    
    SX R_lsole(3, 3), R_rsole(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_lsole(i, j) = T_lsole.rotation()(i, j);
            R_rsole(i, j) = T_rsole.rotation()(i, j);
        }
    }

    // From the force, compute the wrench, then use jacobian.transpose*wrench
    SX F_lsole_heel_l = F_lsole_k(Slice(0, 3));  // Left sole heel left forces
    SX F_lsole_heel_r = F_lsole_k(Slice(3, 6));  // Left sole heel right forces
    SX F_lsole_toe_l  = F_lsole_k(Slice(6, 9));  // Left sole toe left forces
    SX F_lsole_toe_r  = F_lsole_k(Slice(9, 12)); // Left sole toe right forces
    
    SX F_rsole_heel_l = F_rsole_k(Slice(0, 3));  // Right sole heel left forces
    SX F_rsole_heel_r = F_rsole_k(Slice(3, 6));  // Right sole heel right forces
    SX F_rsole_toe_l  = F_rsole_k(Slice(6, 9));  // Right sole toe left forces
    SX F_rsole_toe_r  = F_rsole_k(Slice(9, 12)); // Right sole toe right forces
    
    // Compute the wrench for left sole
    SX heel_pos_l(3, 1);
    SX heel_pos_r(3, 1);
    
    SX toe_pos_l(3, 1);
    SX toe_pos_r(3, 1);


    heel_pos_l(0) = -WBNMPC_params.foot_length/2; heel_pos_l(1) = WBNMPC_params.foot_width/2; heel_pos_l(2) = 0.0; // Heel left position in the local frame
    heel_pos_r(0) = -WBNMPC_params.foot_length/2; heel_pos_r(1) = -WBNMPC_params.foot_width/2; heel_pos_r(2) = 0.0; // Heel right position in the local frame

    toe_pos_l (0) = WBNMPC_params.foot_length/2; toe_pos_l (1) = WBNMPC_params.foot_width/2; toe_pos_l (2) = 0.0; // Toe left position in the local frame
    toe_pos_r (0) = WBNMPC_params.foot_length/2; toe_pos_r (1) = -WBNMPC_params.foot_width/2; toe_pos_r (2) = 0.0; // Toe right position in the local frame
    
    //TODO: Convert this vector to the world frame T_lsole.rotation() * heel_pos_l??
    SX heel_l_w = mtimes(R_lsole, heel_pos_l);
    SX heel_r_w = mtimes(R_lsole, heel_pos_r);
    SX toe_l_w = mtimes(R_lsole, toe_pos_l);
    SX toe_r_w = mtimes(R_lsole, toe_pos_r);

    SX heel_l_w_r = mtimes(R_rsole, heel_pos_l);
    SX heel_r_w_r = mtimes(R_rsole, heel_pos_r);
    SX toe_l_w_r = mtimes(R_rsole, toe_pos_l);
    SX toe_r_w_r = mtimes(R_rsole, toe_pos_r);



    SX wrench_lsole = SX::vertcat({F_lsole_heel_l + F_lsole_heel_r + F_lsole_toe_l + F_lsole_toe_r, 
                                   SX::cross(heel_l_w, F_lsole_heel_l) + SX::cross(heel_r_w, F_lsole_heel_r) + 
                                   SX::cross(toe_l_w, F_lsole_toe_l)   + SX::cross(toe_r_w, F_lsole_toe_r)});

    SX wrench_rsole = SX::vertcat({F_rsole_heel_l + F_rsole_heel_r + F_rsole_toe_l + F_rsole_toe_r, 
                                   SX::cross(heel_l_w_r, F_rsole_heel_l) + SX::cross(heel_r_w_r, F_rsole_heel_r) + 
                                   SX::cross(toe_l_w_r, F_rsole_toe_l)   + SX::cross(toe_r_w_r, F_rsole_toe_r)});

    std::cout << "Wrench left sole" << wrench_lsole << std::endl;
    std::cout << "Wrench right sole" << wrench_rsole << std::endl;

    
    std::cout << "Building dynamic constraint" << std::endl; 
    f_dyn_i = SX::vertcat({SX::zeros(6,1), tau_k}) - tau_RNEA 
                              + mtimes(J_lsole_cs.T(),wrench_lsole)  + mtimes(J_rsole_cs.T(), wrench_rsole) ;// ;

    // f_dyn_i = SX::vertcat({wrench_lsole , wrench_rsole, SX::zeros(num_v_single_step-12,1)});
                                                      
    std::cout << "Done building dynamic constraint" << std::endl;
  //std::cout << "Dyn constraint symbolic" << f_dyn_i << std::endl;
  // f_constraint Dynamic
    f_dyn(Slice(k*num_v_single_step, (k + 1)*num_v_single_step)) = f_dyn_i; // Store the constraint for this step
    std::cout << "Done collecting dynamic constraints for step " << k << std::endl;

  // CONTACT AND SWING CONSTRAINTS
    SX Gam_h_l_k = Gam_h_l(k);
    SX Gam_t_l_k = Gam_t_l(k);
    SX Gam_h_r_k = Gam_h_r(k);
    SX Gam_t_r_k = Gam_t_r(k);
    
    // f_constraint Force when in swing
    f_Fswing(Slice(k*num_force_single_foot_single_step*2,(k+1)*num_force_single_foot_single_step*2))
    = SX::vertcat( {(1-Gam_h_l_k)*F_lsole_heel_l,
                    (1-Gam_h_l_k)*F_lsole_heel_r,
                    (1-Gam_t_l_k)*F_lsole_toe_l,
                    (1-Gam_t_l_k)*F_lsole_toe_r,
                    (1-Gam_h_r_k)*F_rsole_heel_l,
                    (1-Gam_h_r_k)*F_rsole_heel_r,
                    (1-Gam_t_r_k)*F_rsole_toe_l,
                    (1-Gam_t_r_k)*F_rsole_toe_r });

  // f_constraint Friction cone
    f_friction(Slice(k*2*num_contact,(k+1)*2*num_contact))
    = SX::vertcat ( { Gam_h_l_k * (mu*F_lsole_heel_l(2) - SX::sqrt (F_lsole_heel_l(0)*F_lsole_heel_l(0) + F_lsole_heel_l(1)*F_lsole_heel_l(1)+0.0001)),
                      Gam_h_l_k * (mu*F_lsole_heel_r(2) - SX::sqrt (F_lsole_heel_r(0)*F_lsole_heel_r(0) + F_lsole_heel_r(1)*F_lsole_heel_r(1)+0.0001)),
                      Gam_t_l_k * (mu*F_lsole_toe_l(2)  - SX::sqrt (F_lsole_toe_l(0) *F_lsole_toe_l(0)  + F_lsole_toe_l(1)  *F_lsole_toe_l(1)+0.0001)),
                      Gam_t_l_k * (mu*F_lsole_toe_r(2)  - SX::sqrt (F_lsole_toe_r(0) *F_lsole_toe_r(0)  + F_lsole_toe_r(1)  *F_lsole_toe_r(1)+0.0001)),
                      
                      Gam_h_r_k * (mu*F_rsole_heel_l(2) - SX::sqrt (F_rsole_heel_l(0)*F_rsole_heel_l(0) + F_rsole_heel_l(1) *F_rsole_heel_l(1)+0.0001)),
                      Gam_h_r_k * (mu*F_rsole_heel_r(2) - SX::sqrt (F_rsole_heel_r(0)*F_rsole_heel_r(0) + F_rsole_heel_r(1) *F_rsole_heel_r(1)+0.0001)),
                      Gam_t_r_k * (mu*F_rsole_toe_l(2)  - SX::sqrt (F_rsole_toe_l(0) *F_rsole_toe_l(0)  + F_rsole_toe_l(1)  *F_rsole_toe_l(1)+0.0001)),
                      Gam_t_r_k * (mu*F_rsole_toe_r(2)  - SX::sqrt (F_rsole_toe_r(0) *F_rsole_toe_r(0)  + F_rsole_toe_r(1)  *F_rsole_toe_r(1)+0.0001)) } );

    // Reference feet height
    // Find the heel and toe position --> later
    SX heel_l_height = T_lsole.translation()(2);
    SX toe_l_height = T_lsole.translation()(2);
    SX heel_r_height = T_rsole.translation()(2);
    SX toe_r_height = T_rsole.translation()(2);


  // f_constraint Feet height
    f_feet_height (Slice (k*2*2,(k+1)*2*2))
    = SX::vertcat ( { heel_l_height ,
                      toe_l_height  ,
                      heel_r_height ,
                      toe_r_height  });
  


  // f_constraint Tangential contact velocity is zero
  // Extract components
    SX v_linear_l = SX::vertcat({v_lsole_local_world.linear()[0], 
                                  v_lsole_local_world.linear()[1], 
                                  v_lsole_local_world.linear()[2]});
    SX omega_l = SX::vertcat({v_lsole_local_world.angular()[0], 
                               v_lsole_local_world.angular()[1], 
                               v_lsole_local_world.angular()[2]});
    
    SX v_linear_r = SX::vertcat({v_rsole_local_world.linear()[0], 
                                  v_rsole_local_world.linear()[1], 
                                  v_rsole_local_world.linear()[2]});
    SX omega_r = SX::vertcat({v_rsole_local_world.angular()[0], 
                               v_rsole_local_world.angular()[1], 
                               v_rsole_local_world.angular()[2]});
    

    
    // Transform contact positions to world frame and compute velocities

    
    SX v_heel_l_world_l = v_linear_l + SX::cross(omega_l, heel_l_w);
    SX v_heel_r_world_l = v_linear_l + SX::cross(omega_l, heel_r_w);
    SX v_toe_l_world_l = v_linear_l + SX::cross(omega_l, toe_l_w);
    SX v_toe_r_world_l = v_linear_l + SX::cross(omega_l, toe_r_w);
    
    
    SX v_heel_l_world_r = v_linear_r + SX::cross(omega_r, heel_l_w_r);
    SX v_heel_r_world_r = v_linear_r + SX::cross(omega_r, heel_r_w_r);
    SX v_toe_l_world_r = v_linear_r + SX::cross(omega_r, toe_l_w_r);
    SX v_toe_r_world_r = v_linear_r + SX::cross(omega_r, toe_r_w_r);

    f_tang_contact_vel (Slice (k*num_contact*2*2, (k+1)*num_contact*2*2))
    = SX::vertcat ( { Gam_h_l_k * v_heel_l_world_l(0), Gam_h_l_k * v_heel_l_world_l(1),
                      Gam_h_l_k * v_heel_r_world_l(0), Gam_h_l_k * v_heel_r_world_l(1),
                      Gam_t_l_k * v_toe_l_world_l(0),  Gam_t_l_k * v_toe_l_world_l(1),
                      Gam_t_l_k * v_toe_r_world_l(0),  Gam_t_l_k * v_toe_r_world_l(1),
                      Gam_h_r_k * v_heel_l_world_r(0), Gam_h_r_k * v_heel_l_world_r(1),
                      Gam_h_r_k * v_heel_r_world_r(0), Gam_h_r_k * v_heel_r_world_r(1),
                      Gam_t_r_k * v_toe_l_world_r(0),  Gam_t_r_k * v_toe_l_world_r(1),
                      Gam_t_r_k * v_toe_r_world_r(0),  Gam_t_r_k * v_toe_r_world_r(1)} );

    //TODO: separate heel and toe velocity, the correct sequence is toe_l(0-1), heel_l(0-1), toe_r(0-1), heel_r (0-1)
    // Total 6 f_constraint, formulas (4,5,7,8,9,10) in the ref paper.
   }
  
  cs_f_total_constraint_pre = SX::vertcat ( {f_kin,f_dyn,f_Fswing, f_friction, f_feet_height, f_tang_contact_vel} );
  std::cout << "Size of cs_f_total_constraint_pre: "<< cs_f_total_constraint_pre.size1()<<std::endl;
  //cs_f_total_constraint_pre = SX::vertcat ( {f_kin,f_Fswing, f_friction, f_feet_height, f_tang_contact_vel} );

  SX cs_f_total_constraint = cse(cs_f_total_constraint_pre); // Common subexpression elimination

  //cs_f_total_constraint = SX::vertcat ( {f_kin,f_Fswing, f_friction, f_feet_height, f_tang_contact_vel} );
  // cs_f_total_constraint = SX::vertcat ( {f_kin,f_dyn,f_Fswing, f_friction, f_feet_height, f_tang_contact_vel} );
  // cs_f_total_constraint = SX::vertcat ( {f_kin,f_dyn,f_Fswing, f_friction, f_feet_height, f_tang_contact_vel} );
  //cs_f_total_constraint = SX::vertcat ( {f_kin, f_Fswing, f_feet_height,f_tang_contact_vel} );


  



// For the cost function
  SX Jacob_cost_pre = jacobian(cost, decision_vars);
  SX Hessian_cost_pre = hessian(cost, decision_vars);

  SX Jacob_cost =  cse(Jacob_cost_pre);
  SX Hessian_cost = cse(Hessian_cost_pre);


  Function eval_cost = Function("eval_cost", {ca_q, ca_q_r, ca_v, ca_v_r, F_lsole, F_lsole_r, F_rsole, F_rsole_r,
                                              WBNMPC_params.weight_com_xy, WBNMPC_params.weight_com_z, WBNMPC_params.weight_torso, WBNMPC_params.weight_general_qj,
                                              WBNMPC_params.weight_general_vb, WBNMPC_params.weight_general_omega_b, WBNMPC_params.weight_general_v,
                                              WBNMPC_params.weight_contact_force_xy, WBNMPC_params.weight_contact_force_z
                                              }, {cost});
  Function eval_Jacob_cost = Function("eval_Jacobian_cost", {ca_q, ca_q_r, ca_v, ca_v_r, F_lsole, F_lsole_r, F_rsole, F_rsole_r,
                                                            WBNMPC_params.weight_com_xy, WBNMPC_params.weight_com_z, WBNMPC_params.weight_torso, WBNMPC_params.weight_general_qj,
                                                            WBNMPC_params.weight_general_vb, WBNMPC_params.weight_general_omega_b, WBNMPC_params.weight_general_v,
                                                            WBNMPC_params.weight_contact_force_xy, WBNMPC_params.weight_contact_force_z
                                                            }, {Jacob_cost});
  Function eval_Hessian_cost = Function("eval_Hessian_cost", {ca_q_r,
                                                              WBNMPC_params.weight_com_xy, WBNMPC_params.weight_com_z, WBNMPC_params.weight_torso, WBNMPC_params.weight_general_qj,
                                                              WBNMPC_params.weight_general_vb, WBNMPC_params.weight_general_omega_b, WBNMPC_params.weight_general_v,
                                                              WBNMPC_params.weight_contact_force_xy, WBNMPC_params.weight_contact_force_z
                                                              }, {Hessian_cost});

 
// For the initial constraints
  // Function eval_f0 = Function("eval_f0", {ca_q, ca_v}, {f0}); 
  // std::cout << "Done building f0 eval function" << std::endl;

  // SX Jacob_f0 = jacobian(f0, decision_vars);
  // std::ofstream Jacob_f0_file("Jacob_f0.txt");
  // Jacob_f0_file << "Jacob_f0:\n";
  // Jacob_f0_file << std::fixed << std::setprecision(6) << std::setw(3) << Jacob_f0 << std::endl;
  // Function eval_Jacob_f0 = Function("eval_Jacob_f0", {ca_q, ca_v}, {Jacob_f0});

// For the kinemtic constraints  
//   Function eval_f_kin = Function("eval_f_kin", {ca_q, ca_v, dt}, {f_kin});
//   std::ofstream f_kin_file("f_kin.txt");
//   f_kin_file << "f_kin:\n";
//   f_kin_file << f_dyn << std::endl;


//   std::cout << "Done building f_kin eval function" << std::endl;
//   SX Jacob_f_kin = jacobian(f_kin, decision_vars);

//   std::ofstream Jacob_f_kin_file("Jacob_f_kin.txt");
//   Jacob_f_kin_file << "Jacob_f_kin:\n";
//   Jacob_f_kin_file  << Jacob_f_kin << std::endl;
//   Function eval_Jacob_f_kin = Function("eval_Jacob_f_kin", {ca_q, ca_v, dt}, {Jacob_f_kin});

// // For the dynamic constraints
//   Function eval_f_dyn_pre = Function("eval_f_dyn", {ca_q, ca_v, F_lsole, F_rsole, tau, dt}, {f_dyn});
//   std::ofstream f_dyn_file("f_dyn.txt");
//   f_dyn_file << "f_dyn:\n"<< f_dyn << std::endl;
//   std::cout << "Done building f_dyn eval function" << std::endl;
//   SX Jacob_f_dyn = jacobian(f_dyn, decision_vars);
//   std::ofstream Jacob_f_dyn_file("Jacob_f_dyn.txt");
//   Jacob_f_dyn_file << "Jacob_f_dyn:\n";
//   Jacob_f_dyn_file <<  Jacob_f_dyn << std::endl;
//   Function eval_Jacob_f_dyn = Function("eval_Jacob_f_dyn", {ca_q, ca_v, F_lsole, F_rsole, tau, dt}, {Jacob_f_dyn});

// For the total constraints
  Function eval_f_total_constraint = 
  Function ("f_total_constraint", {ca_q, ca_v, F_lsole, F_rsole, tau, dt, mu, Gamma },{cs_f_total_constraint});
  std::ofstream f_total_constraint_file("f_total_constraint.txt");
  f_total_constraint_file << "f_total_constraint: (" << cs_f_total_constraint.size1() << "," << cs_f_total_constraint.size2() << "): \n"  << cs_f_total_constraint << std::endl;
  std::cout << "Done building f_total_constraint eval function" << std::endl;

  SX cs_Jacob_f_total_constraint_pre = jacobian(cs_f_total_constraint, decision_vars);
  SX cs_Jacob_f_total_constraint = cse(cs_Jacob_f_total_constraint_pre); // Common subexpression elimination

  std::ofstream Jacob_f_total_constraint_file("Jacob_f_total_constraint.txt");
  Jacob_f_total_constraint_file << "Jacob_f_total_constraint: \n" << cs_Jacob_f_total_constraint <<  std::endl;

  Function eval_Jacob_f_total_constraint = 
  Function("Jacob_f_total_constraint", {ca_q, ca_v, F_lsole, F_rsole, tau, dt, mu, Gamma}, {cs_Jacob_f_total_constraint});
  // The string inside the Function is the name of the function in the codegen file
  std::cout << "Done building Jacob_f_total_constraint eval function" << std::endl;

  #if codegen_enabled
  
  Dict codegen_options;
  codegen_options["with_header"] = true;
  codegen_options["cpp"] = false;
  //codegen_options["cse"] = true;
  // eval_Jacob_cost.generate("eval_Jacob_code.cpp", codegen_options);

  CodeGenerator Codegen("eval_codegen_func.c", codegen_options);
  std::cout << "Generating code for the functions" << std::endl;
  Codegen.add(eval_cost);
  Codegen.add(eval_Jacob_cost);
  Codegen.add(eval_Hessian_cost);
  
  Codegen.add(eval_f_total_constraint);
  Codegen.add(eval_Jacob_f_total_constraint);
  // Codegen.add_include("codegen_func.hpp",true, "~/Sapienza/Excelent/mpc/include");
  Codegen.generate();
#endif

labrob::pressAnyKey();

  // Prepare params for the codegen API


  





  // Config the mujoco simulator
  auto& mujoco_ui = *labrob::MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

  for (int i = 0; i < mj_model_ptr->nu; ++i) {
    int joint_id = mj_model_ptr->actuator_trnid[i * 2];
    std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
    joint_names_log_file << joint_name << std::endl;
  }

  joint_names_log_file.flush();
  joint_names_log_file.close();

  static int framerate = 60.0;

  // Simulation loop:
  while (!mujoco_ui.windowShouldClose()) {

    auto start_time = std::chrono::high_resolution_clock::now();

    mjtNum simstart = mj_data_ptr->time;
    while( mj_data_ptr->time - simstart < 1.0/framerate ) {
      
      labrob::RobotState robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);

      // // Update walking manager:
      // labrob::JointCommand joint_command;
      // walking_manager.update(robot_state, joint_command);
      
      mj_step1(mj_model_ptr, mj_data_ptr);
      robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
      //std::cout << "Joint configuration measured from mujoco: " << std::endl;
      q_meas = labrob::robot_state_to_pinocchio_joint_configuration(robot_model_test,robot_state);
      //v_meas = labrob::robot_state_to_pinocchio_joint_velocity(robot_model_test,robot_state);

      std::cout << v_meas.transpose() << std::endl;
      //labrob::pressAnyKey();
      std::vector<double> q_meas_i((size_t)num_q);
      std::vector<double> v_meas_i((size_t)num_v);
      std::vector<double> q_ref_i((size_t)num_q);
      std::vector<double> v_ref_i((size_t)num_v);

      std::vector<double> F_lsole_vec((size_t)num_force);
      std::vector<double> F_rsole_vec((size_t)num_force);

      std::vector<double> F_lsole_r_vec((size_t)num_force);
      std::vector<double> F_rsole_r_vec((size_t)num_force);

      std::vector<double> tau_vec((size_t)num_torques);
      std::vector<double> Gamma_vec(1.0,(size_t)4*N);
      std::vector<double> ref_feet_height_vec(4*N);



      for (int i = 0; i < N; ++i) {
        Eigen::Map<pinocchio::Model::ConfigVectorType>(q_meas_i.data()+i*robot_model_test.nq, robot_model_test.nq, 1) = q_meas;
        Eigen::Map<pinocchio::Model::ConfigVectorType>(q_ref_i.data()+i*robot_model_test.nq, robot_model_test.nq, 1) = q_init;
        Eigen::Map<pinocchio::Model::TangentVectorType>(v_meas_i.data()+i*robot_model_test.nv, robot_model_test.nv, 1) = v_meas;
        Eigen::Map<pinocchio::Model::TangentVectorType>(v_ref_i.data()+i*robot_model_test.nv, robot_model_test.nv, 1) = Eigen::VectorXd::Zero(robot_model_test.nv);
        Eigen::Map<Eigen::VectorXd>(F_lsole_vec.data()+i*num_force_single_foot_single_step, num_force_single_foot_single_step, 1) = Eigen::VectorXd::Ones(num_force_single_foot_single_step);
        Eigen::Map<Eigen::VectorXd>(F_rsole_vec.data()+i*num_force_single_foot_single_step, num_force_single_foot_single_step, 1) = Eigen::VectorXd::Ones(num_force_single_foot_single_step);
        Eigen::Map<Eigen::VectorXd>(F_lsole_r_vec.data()+i*num_force_single_foot_single_step, num_force_single_foot_single_step, 1) = Eigen::VectorXd::Zero(num_force_single_foot_single_step);
        Eigen::Map<Eigen::VectorXd>(F_rsole_r_vec.data()+i*num_force_single_foot_single_step, num_force_single_foot_single_step, 1) = Eigen::VectorXd::Zero(num_force_single_foot_single_step);
        Eigen::Map<Eigen::VectorXd>(tau_vec.data()+i*num_torques_single_step, num_torques_single_step, 1) = Eigen::VectorXd::Ones(num_torques_single_step);
      }
      //std::cout << "q pos iterration: " << q_meas_i << std::endl;

      // DM cost_Jacob = eval_Jacob_cost(
      //     DMVector{q_meas_i, q_ref_i, v_meas_i, v_ref_i, F_lsole_vec, F_lsole_r_vec, F_rsole_vec, F_rsole_r_vec})[0];
      // //std::cout << "Jacobian of the cost function size: " << cost_Jacob.size1() << "," << cost_Jacob.size2() << std::endl;
      // DM cost_Hessian = eval_Hessian_cost(
      //     DMVector{q_meas_i, q_ref_i, v_meas_i, v_ref_i, F_lsole_vec, F_lsole_r_vec, F_rsole_vec, F_rsole_r_vec})[0];

      // DM f0_eval = eval_f0(DMVector{q_meas_i, v_meas_i})[0];
      // DM f0_Jacob_eval = eval_Jacob_f0(DMVector{q_meas_i, v_meas_i})[0];

      // DM f_kin_eval = eval_f_kin(DMVector{q_meas_i, v_meas_i, 0.001})[0];
      // DM f_kin_Jacob_eval = eval_Jacob_f_kin(DMVector{q_meas_i, v_meas_i, 0.001})[0];

      // DM f_dyn_eval = eval_f_dyn_pre(DMVector{q_meas_i, v_meas_i, F_lsole_vec, F_rsole_vec, tau_vec, 0.001})[0];
      // std::cout << "f_dyn_pre" << std::endl << f_dyn_eval << std::endl;
      // DM Jacob_f_dyn_eval = eval_Jacob_f_dyn(DMVector{q_meas_i, v_meas_i, F_lsole_vec, F_rsole_vec, tau_vec, 0.001})[0];
      
      // std::ofstream Jacob_f_dyn_eval_file("Jacob_f_dyn_eval.txt");
      // Jacob_f_dyn_eval_file << "Jacob_f_dyn_eval:\n";
      // Jacob_f_dyn_eval.print_dense(Jacob_f_dyn_eval_file,false);
      // Jacob_f_dyn_eval_file << std::endl;

      // std::vector<DM> data_meas = {q_meas_i, v_meas_i, F_lsole_vec, F_rsole_vec, tau_vec, 0.001};
      // std::vector<std::vector <double> > data_meas;

      //DM DM_Jacob_f_total_constraint = eval_Jacob_f_total_constraint(DMVector{q_meas_i, v_meas_i, F_lsole_vec, F_rsole_vec, tau_vec, 0.001, 0.5, Gamma_vec})[0];

      // std::ofstream Jacob_f_constr_eval_file("Jacob_f_constr_eval.txt");
      // Jacob_f_constr_eval_file << "Jacob_f_constr_eval:\n";
      // DM_Jacob_f_total_constraint.print_dense(Jacob_f_constr_eval_file,false);
      // Jacob_f_constr_eval_file << std::endl;

      const casadi_real* data_meas[8];
      data_meas[0] = q_meas_i.data();
      data_meas[1] = v_meas_i.data();
      data_meas[2] = F_lsole_vec.data();
      data_meas[3] = F_rsole_vec.data();
      data_meas[4] = tau_vec.data();
      data_meas[5] = d_dt_arr;
      data_meas[6] = d_mu_arr;

    
      data_meas[7] = Gamma_vec.data();
      //data_meas[8] = ref_feet_height_vec.data();
      
      labrob::qpsolvers::CSCMatrix_params res_out;
      std::cout << "Codegen fconstraint" << std::endl;

      //eval_codegen(f_total_constraint_work, f_total_constraint, f_total_constraint_sparsity_out, data_meas,res_out );
      std::cout << "f_fconstraint_eval: " << std::endl;

      // for (int i = 0; i < num_constraint; ++i) {
      //   std::cout << res_out[0][i] << " ";
      // }
      // std::cout << std::endl;


      // casadi_real* Jacob_res_out[1];
      // std::cout << "Codegen Jacob fconstraint:" << std::endl;

      // eval_codegen(Jacob_f_total_constraint_work, Jacob_f_total_constraint, data_meas,Jacob_res_out );
      // std::cout << "Jacob_f_constraint_eval: " << std::endl;

      // for (int i = 0; i < num_constraint; ++i) {
      //   std::cout << Jacob_res_out[0][i] << " ";
      // }


      // std::cout << "f0 size: " << f0_eval.size1() << "," << f0_eval.size2() << std::endl;

      // std::cout << "Jacobian of the cost function: " << cost_Jacob << std::endl;
      // std::cout << "Hessian of the cost function: " << cost_Hessian << std::endl;    
      
      //std::cout << "Joint velocity measured from mujoco: " << std::endl;
      //std::cout << v_meas.transpose() << std::endl;


      // for (int i = 0; i < mj_model_ptr->nu; ++i) {
      //   int joint_id = mj_model_ptr->actuator_trnid[i * 2];
      //   std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
      //   int jnt_qvel_idx = mj_model_ptr->jnt_dofadr[joint_id];
      //   mj_data_ptr->ctrl[i] = joint_command[joint_name];

      //   joint_vel_log_file << mj_data_ptr->qvel[jnt_qvel_idx] << " ";
      //   joint_eff_log_file << mj_data_ptr->ctrl[i] << " ";
      // }

      mj_step2(mj_model_ptr, mj_data_ptr);

      joint_vel_log_file << std::endl;
      joint_eff_log_file << std::endl;
    
    }

    // Fine misurazione del tempo
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Stampa del tempo di esecuzione
    std::cout << "Tempo di esecuzione del main: " << duration << " millisecondi" << std::endl;


    mujoco_ui.render();
  }

  // Free memory (Mujoco):
  mj_deleteData(mj_data_ptr);
  mj_deleteModel(mj_model_ptr);

  joint_vel_log_file.close();
  joint_eff_log_file.close();

  return 0;
}



//// main to test the QP solver




// int main() {
//     int num_state = 3;
//     int num_contrl = 1;
//     int num_variables = num_state + num_contrl;
//     int num_eq_constraints = 1;
//     int num_ineq_constraints = 2;

//     Eigen::MatrixXd H(num_variables, num_variables);
//     Eigen::VectorXd f(num_variables);
//     Eigen::MatrixXd A(num_eq_constraints, num_variables);
//     Eigen::VectorXd b(num_eq_constraints);
//     Eigen::MatrixXd C(num_ineq_constraints, num_variables);
//     Eigen::VectorXd d_min(num_ineq_constraints);
//     Eigen::VectorXd d_max(num_ineq_constraints);

//     H << 6, 0, 0, 0,
//          0, 4, 0, 0,
//          0, 0, 7, 0,
//          0, 0, 0, 9;

//     f << 1, 2, 3, 4;
//     A << 4, 0, 0, 0;
//     b << 1;
//     C << 1, 1, 1, 1,
//          2, 2, 2, 2;
//     d_min << 0, 0;
//     d_max << 2, 2;
        
//     qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
//       std::make_shared<labrob::qpsolvers::OSQPSolver>(num_variables, num_eq_constraints, num_ineq_constraints));

//     qp_solver_ptr_->solve(
//         H, f, A, b, C, d_min, d_max);

//     std::cerr<< "Solution: " << std::endl << qp_solver_ptr_->get_solution()<< std::endl;

    
    
// }