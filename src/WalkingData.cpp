#include <hrp4_locomotion/WalkingData.hpp>

#include <iostream>

namespace labrob {

const labrob::WalkingState&
WalkingData::getWalkingState() const {
  return footstep_plan.front().getWalkingState();  
}

void
WalkingData::updateFootstepPlanWithCurrentStance(
    const labrob::SE3& leftFootConfiguration,
    const labrob::SE3& rightFootConfiguration
) {
  size_t idx = 0;

  // Update all elements up to the first single support (sharing both feet):
  bool first_single_support_found = false;
  while (idx < footstep_plan.size() && !first_single_support_found) {
    if (footstep_plan[idx].getWalkingState() == labrob::WalkingState::SingleSupport) {
      first_single_support_found = true;
    }
    footstep_plan[idx].getFeetPlacement().setFeetConfiguration(
        leftFootConfiguration,
        rightFootConfiguration
    );
    ++idx;
  }

  // Update all elements up to the second single support (sharing only one foot):
  while (idx < footstep_plan.size()) {
    if (footstep_plan[idx].getWalkingState() != labrob::WalkingState::SingleSupport) {
      if (footstep_plan[idx].getFeetPlacement().getSupportFoot() == labrob::Foot::LEFT) {
        footstep_plan[idx].getFeetPlacement().setLeftFootConfiguration(
            leftFootConfiguration
        );
      } else {
        footstep_plan[idx].getFeetPlacement().setRightFootConfiguration(
            rightFootConfiguration
        );
      }
    } else {
      break;
    }
    ++idx;
  }

}

void
WalkingData::updateWalkingState(int64_t t) {
  if (getWalkingState() == labrob::WalkingState::Init) {
    footstep_plan.front().setWalkingState(labrob::WalkingState::PostureRegulation);
    t0 = t;
  } else if (getWalkingState() == labrob::WalkingState::Standing && footstep_plan.size() == 1) {
    // Update t0 to keep robot in standing position.
    t0 = t;
  } 
  else if (t >= t0 + footstep_plan.front().getDuration()) {
    footstep_plan.pop_front();
    t0 = t;
  }
}

} // end namespace labrob