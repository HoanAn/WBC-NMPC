#ifndef LABROB_LIP_SIMULATOR
#define LABROB_LIP_SIMULATOR

#include <filesystem>

#include <hrp4_locomotion/FootstepPlanElement.hpp>
#include <hrp4_locomotion/LIPState.hpp>
#include <hrp4_locomotion/WalkingData.hpp>


namespace labrob {

struct LIPSimulatorOptions {
  int64_t timestep_msec;
  int64_t duration_msec;
  double eta;
  LIPState initial_lip_state;
  std::deque<labrob::FootstepPlanElement> footstep_plan;
  // ISMPC options:
  int64_t mpc_prediction_horizon_msec;
  int64_t mpc_timestep_msec;
  double mpc_com_target_height;
  double mpc_foot_constraint_square_length;
  double mpc_foot_constraint_square_height;
  // Log options:
  std::filesystem::path log_directory_path;
};

class LIPSimulator {
 public:
  LIPSimulator(const LIPSimulatorOptions& options);

  void run();

 private:
  LIPSimulatorOptions options_;
};

} // end namespace labrob

#endif // LABROB_LIP_SIMULATOR
