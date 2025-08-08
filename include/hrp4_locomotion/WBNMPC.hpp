// Created by An Nguyen 03/07/2025 

#ifndef LABROB_WBNMPC_HPP_
#define LABROB_WBNMPC_HPP_

// STL
#include <fstream>
#include <memory>
#include <vector>



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

#include <casadi/casadi.hpp>


#include <hrp4_locomotion/LIPState.hpp>
#include <hrp4_locomotion/WalkingData.hpp>
#include <hrp4_locomotion/GaitConfiguration.hpp>
#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/RobotState.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>


// Eigen
// #include <Eigen/Core>
// #include <Eigen/Geometry>





namespace labrob {

using CasadiData = pinocchio::DataTpl<casadi::SX>;
using CasadiModel = pinocchio::ModelTpl<casadi::SX>;
using namespace casadi;

using Ca_ConfigVector = CasadiData::ConfigVectorType;
using Ca_TangentVector = CasadiData::TangentVectorType;

struct WholeBodyMPCParams {
  double Kp_motion;
  double Kd_motion;
  double Kp_regulation;
  double Kd_regulation;

  double weight_q_ddot;
  
  double weight_lsole;
  double weight_rsole;
  
  double weight_pelvis;
  double weight_regulation;
  double weight_angular_momentum;
  /// Weight needed
  double weight_com;
  double weight_torso;
  double weight_general_qj; //generalized configuration q = [p_b, \theta_b, q_j] \in R^{n_j +6}
  
  double weight_general_vb;
  double weight_general_omega_b;
  double weight_general_v; //generalized velocity v = [vb ; \omega_b, \dot q_j]
  
  double weight_contact_force;
  //
  double cmm_selection_matrix_x;
  double cmm_selection_matrix_y;
  double cmm_selection_matrix_z;

  double gamma;

  double mu;

  double foot_length;
  double foot_width;

  static WholeBodyMPCParams getDefaultParams();
};

class WholeBodyMPC {
 public:
  WholeBodyMPC(const WholeBodyMPCParams& params,
                            const pinocchio::Model& robot_model,
                            const Eigen::VectorXd& q_jnt_reg,
                            double sample_time,
                            std::map<std::string, double>& armature);

  labrob::JointCommand
  compute_inverse_dynamics(
      const pinocchio::Model& robot_model,
      const labrob::RobotState& robot_state,
      pinocchio::Data& robot_data,
      const labrob::GaitConfiguration& current,
      const labrob::GaitConfiguration& desired
  );

 private:
  pinocchio::Model robot_model_;
  pinocchio::Data robot_data_;

  pinocchio::FrameIndex lsole_idx_;
  pinocchio::FrameIndex rsole_idx_;
  pinocchio::FrameIndex torso_idx_;
  pinocchio::FrameIndex pelvis_idx_;

  Eigen::MatrixXd J_torso_;
  Eigen::MatrixXd J_pelvis_;
  Eigen::MatrixXd J_lsole_;
  Eigen::MatrixXd J_rsole_;

  Eigen::MatrixXd J_torso_dot_;
  Eigen::MatrixXd J_pelvis_dot_;
  Eigen::MatrixXd J_lsole_dot_;
  Eigen::MatrixXd J_rsole_dot_;

  Eigen::VectorXd q_jnt_reg_;

  double sample_time_;

  WholeBodyMPCParams params_;

  Eigen::VectorXd M_armature_;

  int n_joints_;
  int n_contacts_;
  int n_wbc_variables_;
  int n_wbc_equalities_;
  int n_wbc_inequalities_;

  std::unique_ptr<qpsolvers::QPSolverEigenWrapper<double>> wbc_solver_ptr_;
  Function eval_Jacob_rnea_;
};

}

#endif // LABROB_WBNMPC_HPP_