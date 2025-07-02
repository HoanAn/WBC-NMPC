#ifndef LABROB_FOOTSTEP_PLAN_ELEMENT_HPP_
#define LABROB_FOOTSTEP_PLAN_ELEMENT_HPP_

#include <vector>

#include <hrp4_locomotion/DoubleSupportConfiguration.hpp>
#include <hrp4_locomotion/WalkingState.hpp>

namespace labrob {

class FootstepPlanElement {
 public:
  FootstepPlanElement(
      const labrob::DoubleSupportConfiguration& feet_placement,
      double swing_foot_trajectory_height,
      int64_t duration,
      const labrob::WalkingState& walking_state
  );
  
  labrob::DoubleSupportConfiguration& getFeetPlacement();
  const labrob::DoubleSupportConfiguration& getFeetPlacement() const;

  double getSwingFootTrajectoryHeight() const;

  int64_t getDuration() const;

  const labrob::WalkingState& getWalkingState() const;

  void
  setFeetPlacement(const labrob::DoubleSupportConfiguration& feet_placement);

  void setSwingFootTrajectoryHeight(double swing_foot_trajectory_height);

  void setDuration(int64_t duration);

  void setWalkingState(const labrob::WalkingState& walking_state); 


 protected:
  labrob::DoubleSupportConfiguration feet_placement_;
  labrob::WalkingState walking_state_;
  double swing_foot_trajectory_height_;
  int64_t duration_;
}; // end class FootstepPlanElement

} // end namespace labrob

#endif // LABROB_FOOTSTEP_PLAN_ELEMENT_HPP_