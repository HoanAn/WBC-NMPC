//
// Created by mmaximo on 20/02/24.
//

#ifndef LABROB_WHOLE_BODY_CONTROLLER_H_
#define LABROB_WHOLE_BODY_CONTROLLER_H_

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
#include <hrp4_locomotion/RobotState.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>

namespace labrob {

struct WholeBodyControllerParams {
  double Kp_motion;
  double Kd_motion;
  double Kp_regulation;
  double Kd_regulation;

  double weight_q_ddot;
  double weight_com;
  double weight_lsole;
  double weight_rsole;
  double weight_torso;
  double weight_pelvis;
  double weight_regulation;
  double weight_angular_momentum;

  double cmm_selection_matrix_x;
  double cmm_selection_matrix_y;
  double cmm_selection_matrix_z;

  double gamma;

  double mu;

  double foot_length;
  double foot_width;

  static WholeBodyControllerParams getDefaultParams();
};

class WholeBodyController {
 public:
  WholeBodyController(const WholeBodyControllerParams& params,
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

  WholeBodyControllerParams params_;

  Eigen::VectorXd M_armature_;

  int n_joints_;
  int n_contacts_;
  int n_wbc_variables_;
  int n_wbc_equalities_;
  int n_wbc_inequalities_;

  std::unique_ptr<qpsolvers::QPSolverEigenWrapper<double>> wbc_solver_ptr_;

};

}

#endif //LABROB_WHOLE_BODY_CONTROLLER_H_
