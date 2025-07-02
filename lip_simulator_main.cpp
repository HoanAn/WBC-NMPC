#include <Eigen/Core>

#include <hrp4_locomotion/DoubleSupportConfiguration.hpp>
#include <hrp4_locomotion/LIPSimulator.hpp>
#include <hrp4_locomotion/LIPState.hpp>
#include <hrp4_locomotion/utils.hpp>

int main() {
  labrob::LIPSimulatorOptions lip_simulator_options;
  lip_simulator_options.timestep_msec = 10;
  Eigen::Vector3d p_CoM(0.0, 0.0, 0.8);
  Eigen::Vector3d v_CoM(0.0, 0.0, 0.0);
  Eigen::Vector3d p_ZMP(0.0, 0.0, 0.0);
  lip_simulator_options.initial_lip_state = labrob::LIPState(p_CoM, v_CoM, p_ZMP);
  lip_simulator_options.eta = std::sqrt(9.81 / (p_CoM.z() - p_ZMP.z()));
  lip_simulator_options.mpc_prediction_horizon_msec = 2000;
  lip_simulator_options.mpc_timestep_msec = 100;
  lip_simulator_options.mpc_com_target_height = p_CoM.z() - p_ZMP.z();
  lip_simulator_options.mpc_foot_constraint_square_length = 0.05;
  lip_simulator_options.mpc_foot_constraint_square_height = 0.05;
  lip_simulator_options.log_directory_path = "/tmp/labrob/lip_simulator";
  // TODO: read footstep plan from .json file.
  auto R_lsole_init = Eigen::Matrix3d::Identity();
  auto p_lsole_init = Eigen::Vector3d(0.0, -0.2, 0.0);
  auto R_rsole_init = Eigen::Matrix3d::Identity();
  auto p_rsole_init = Eigen::Vector3d(0.0, 0.2, 0.0);
  double step_length_x = 0.2;
  double step_length_y = 0.0;
  double step_rotation = 0.0;
  int n_steps = 10;
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(R_lsole_init, p_lsole_init),
          labrob::SE3(R_rsole_init, p_rsole_init),
          labrob::Foot::RIGHT
      ),
      0.0,
      2000,
      labrob::WalkingState::Init
  ));
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(R_lsole_init, p_lsole_init),
          labrob::SE3(R_rsole_init, p_rsole_init),
          labrob::Foot::RIGHT
      ),
      0.0,
      2000,
      labrob::WalkingState::Standing
  ));

  double double_support_duration = 600;
  double single_support_duration = 600;
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(R_lsole_init, p_lsole_init),
          labrob::SE3(R_rsole_init, p_rsole_init),
          labrob::Foot::RIGHT
      ),
      0.0,
      double_support_duration,
      labrob::WalkingState::Starting
  ));
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(R_lsole_init, p_lsole_init),
          labrob::SE3(R_rsole_init, p_rsole_init),
          labrob::Foot::RIGHT
      ),
      0.0,
      single_support_duration,
      labrob::WalkingState::SingleSupport
  ));
  for (int n = 0; n < n_steps; n += 2) {
    lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * R_lsole_init, p_lsole_init + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz(n * step_rotation) * R_rsole_init, p_rsole_init + n * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::RIGHT
        ),
        0.0,
        double_support_duration,
        labrob::WalkingState::DoubleSupport
    ));
    lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * R_lsole_init, p_lsole_init + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz(n * step_rotation) * R_rsole_init, p_rsole_init + n * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::LEFT
        ),
        0.0,
        single_support_duration,
        labrob::WalkingState::SingleSupport
    ));
    lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * R_lsole_init, p_lsole_init + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz((n + 2) * step_rotation) * R_rsole_init, p_rsole_init + (n + 2) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::LEFT
        ),
        0.0,
        double_support_duration,
        labrob::WalkingState::DoubleSupport
    ));
    lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(labrob::Rz((n + 1) * step_rotation) * R_lsole_init, p_lsole_init + (n + 1) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::SE3(labrob::Rz((n + 2) * step_rotation) * R_rsole_init, p_rsole_init + (n + 2) * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
            labrob::Foot::RIGHT
        ),
        0.0,
        single_support_duration,
        labrob::WalkingState::SingleSupport
    ));
  }
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * R_lsole_init, p_lsole_init + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * R_rsole_init, p_rsole_init + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::Foot::RIGHT
      ),
      0.0,
      700,
      labrob::WalkingState::Stopping
  ));
  lip_simulator_options.footstep_plan.push_back(labrob::FootstepPlanElement(
      labrob::DoubleSupportConfiguration(
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * R_lsole_init, p_lsole_init + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::SE3(labrob::Rz(n_steps * step_rotation) * R_rsole_init, p_rsole_init + n_steps * Eigen::Vector3d(step_length_x, step_length_y, 0.0)),
          labrob::Foot::RIGHT
      ),
      0.0,
      2000,
      labrob::WalkingState::Standing
  ));
  lip_simulator_options.duration_msec = 0;
  for (const auto& footstep_plan_elem : lip_simulator_options.footstep_plan) {
    lip_simulator_options.duration_msec += footstep_plan_elem.getDuration();
  }
  
  labrob::LIPSimulator lip_simulator(lip_simulator_options);
  lip_simulator.run();

  return 0;
}