#include <hrp4_locomotion/FootstepPlanElement.hpp>

namespace labrob {

FootstepPlanElement::FootstepPlanElement(
    const labrob::DoubleSupportConfiguration& feet_placement,
    double swing_foot_trajectory_height,
    int64_t duration,
    const labrob::WalkingState& walking_state
) : feet_placement_(feet_placement),
    walking_state_(walking_state),
    swing_foot_trajectory_height_(swing_foot_trajectory_height),
    duration_(duration)
     { }

labrob::DoubleSupportConfiguration&
FootstepPlanElement::getFeetPlacement() {
  return feet_placement_;
}

const labrob::DoubleSupportConfiguration&
FootstepPlanElement::getFeetPlacement() const {
  return feet_placement_;
}

double
FootstepPlanElement::getSwingFootTrajectoryHeight() const {
  return swing_foot_trajectory_height_;
}

int64_t
FootstepPlanElement::getDuration() const {
  return duration_;
}

const labrob::WalkingState&
FootstepPlanElement::getWalkingState() const {
  return walking_state_;
}

void
FootstepPlanElement::setFeetPlacement(
    const labrob::DoubleSupportConfiguration& feet_placement) {
  feet_placement_ = feet_placement;
}

void
FootstepPlanElement::setSwingFootTrajectoryHeight(
    double swing_foot_trajectory_height) {
  swing_foot_trajectory_height_ = swing_foot_trajectory_height;
}

void
FootstepPlanElement::setDuration(int64_t duration) {
  duration_ = duration;
}

void
FootstepPlanElement::setWalkingState(
    const labrob::WalkingState& walking_state) {
  walking_state_ = walking_state;
}

} // end namespace labrob